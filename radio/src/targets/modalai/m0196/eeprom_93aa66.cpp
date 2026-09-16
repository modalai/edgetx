/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#include "eeprom_93aa66.h"

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "delays_driver.h"
#include "hal.h"
#include "hal/gpio.h"
#include "hal/watchdog_driver.h"

namespace {

constexpr uint32_t EEPROM_MAX_CLOCK = 2000000;
constexpr uint16_t COMMAND_START = 1u << 11;
constexpr uint16_t COMMAND_READ = COMMAND_START | (2u << 9);
constexpr uint16_t COMMAND_WRITE = COMMAND_START | (1u << 9);
constexpr uint16_t COMMAND_EWEN = COMMAND_START | (3u << 7);
constexpr uint16_t COMMAND_EWDS = COMMAND_START;
constexpr uint32_t TRANSFER_TIMEOUT = 100000;
constexpr uint8_t WRITE_TIMEOUT_MS = 20;
constexpr uint8_t CHIP_SELECT_LOW_US = 1;

const stm32_spi_t eepromSpi = {
  .SPIx = HELM_EEPROM_SPI,
  .SCK = HELM_EEPROM_SCK,
  .MISO = HELM_EEPROM_MISO,
  .MOSI = HELM_EEPROM_MOSI,
  .CS = HELM_EEPROM_CS,
  .DMA = nullptr,
  .txDMA_PeriphRequest = 0,
  .rxDMA_PeriphRequest = 0,
  .txDMA_Stream = 0,
  .rxDMA_Stream = 0,
  .DMA_FIFOMode = 0,
  .DMA_FIFOThreshold = 0,
  .DMA_MemoryOrM2MDstDataSize = 0,
  .DMA_MemBurst = 0,
};

bool initialized;

void deselectEeprom()
{
  gpio_clear(eepromSpi.CS);
  delay_us(CHIP_SELECT_LOW_US);
}

// SPI6 supports a maximum frame width of 16 bits.
bool transferFrame(uint16_t output, uint32_t width,
                   uint16_t* input = nullptr)
{
  auto* spi = eepromSpi.SPIx;
  LL_SPI_SetDataWidth(spi, width);
  LL_SPI_SetTransferSize(spi, 1);
  LL_SPI_Enable(spi);
  LL_SPI_StartMasterTransfer(spi);

  uint32_t timeout = TRANSFER_TIMEOUT;
  while (!LL_SPI_IsActiveFlag_TXP(spi) && --timeout) {
  }
  if (!timeout) goto error;
  if (width <= LL_SPI_DATAWIDTH_8BIT) {
    LL_SPI_TransmitData8(spi, static_cast<uint8_t>(output));
  } else {
    LL_SPI_TransmitData16(spi, output);
  }

  timeout = TRANSFER_TIMEOUT;
  while (!LL_SPI_IsActiveFlag_RXP(spi) && --timeout) {
  }
  if (!timeout) goto error;
  if (width <= LL_SPI_DATAWIDTH_8BIT) {
    const uint8_t value = LL_SPI_ReceiveData8(spi);
    if (input) *input = value;
  } else {
    const uint16_t value = LL_SPI_ReceiveData16(spi);
    if (input) *input = value;
  }

  timeout = TRANSFER_TIMEOUT;
  while (!LL_SPI_IsActiveFlag_EOT(spi) && --timeout) {
  }
  if (!timeout) goto error;

  LL_SPI_ClearFlag_EOT(spi);
  LL_SPI_ClearFlag_TXTF(spi);
  LL_SPI_Disable(spi);
  return true;

error:
  LL_SPI_Disable(spi);
  return false;
}

bool transferCommand(uint16_t command)
{
  gpio_set(eepromSpi.CS);
  const bool success =
      transferFrame(command, LL_SPI_DATAWIDTH_12BIT);
  deselectEeprom();
  return success;
}

bool setWriteEnable(bool enabled)
{
  return transferCommand(enabled ? COMMAND_EWEN : COMMAND_EWDS);
}

bool waitForWrite()
{
  for (uint8_t elapsed = 0; elapsed < WRITE_TIMEOUT_MS; ++elapsed) {
    gpio_set(eepromSpi.CS);
    delay_us(1);
    const bool ready = gpio_read(eepromSpi.MISO) != 0;
    deselectEeprom();
    if (ready) return true;
    delay_ms(1);
  }
  return false;
}

bool writeByte(uint16_t address, uint8_t value)
{
  gpio_set(eepromSpi.CS);
  const bool success =
      transferFrame(COMMAND_WRITE | address, LL_SPI_DATAWIDTH_12BIT) &&
      transferFrame(value, LL_SPI_DATAWIDTH_8BIT);
  deselectEeprom();
  return success && waitForWrite();
}

}  // namespace

void helmEepromInit()
{
  stm32_spi_init(&eepromSpi, LL_SPI_DATAWIDTH_12BIT);
  stm32_spi_set_max_baudrate(&eepromSpi, EEPROM_MAX_CLOCK);
  LL_SPI_SetClockPolarity(eepromSpi.SPIx, LL_SPI_POLARITY_LOW);
  LL_SPI_SetClockPhase(eepromSpi.SPIx, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetTransferBitOrder(eepromSpi.SPIx, LL_SPI_MSB_FIRST);
  gpio_clear(eepromSpi.CS);
  initialized = true;
}

bool helmEepromRead(uint16_t address, void* data, size_t size)
{
  if (!initialized || !data || address > HELM_EEPROM_SIZE ||
      size > static_cast<size_t>(HELM_EEPROM_SIZE - address)) {
    return false;
  }

  auto* output = static_cast<uint8_t*>(data);
  for (size_t index = 0; index < size; ++index) {
    uint16_t input = 0;
    gpio_set(eepromSpi.CS);
    // The first EEPROM read bit is a dummy zero.
    const bool success =
        transferFrame(COMMAND_READ | (address + index),
                      LL_SPI_DATAWIDTH_12BIT) &&
        transferFrame(0, LL_SPI_DATAWIDTH_9BIT, &input);
    deselectEeprom();
    if (!success) return false;
    output[index] = static_cast<uint8_t>(input);
  }
  return true;
}

bool helmEepromWrite(uint16_t address, const void* data, size_t size)
{
  if (!initialized || !data || address > HELM_EEPROM_SIZE ||
      size > static_cast<size_t>(HELM_EEPROM_SIZE - address)) {
    return false;
  }
  if (size == 0) return true;
  if (!setWriteEnable(true)) return false;

  const auto* input = static_cast<const uint8_t*>(data);
  bool success = true;
  for (size_t index = 0; index < size; ++index) {
    WDG_RESET();
    if (!writeByte(address + index, input[index])) {
      success = false;
      break;
    }
  }

  if (!setWriteEnable(false)) success = false;
  return success;
}
