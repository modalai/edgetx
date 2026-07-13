/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License,
 * version 2.
 */

#include "hal/gpio.h"
#include "stm32_dma.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "board.h"
#include "lcd.h"
#include "timers_driver.h"

#include <cstring>

#if !defined(BOOT)
#include "edgetx.h"
#endif

#define LCD_NCS_HIGH() gpio_set(LCD_NCS_GPIO)
#define LCD_NCS_LOW() gpio_clear(LCD_NCS_GPIO)
#define LCD_A0_HIGH() gpio_set(LCD_A0_GPIO)
#define LCD_A0_LOW() gpio_clear(LCD_A0_GPIO)
#define LCD_RST_HIGH() gpio_set(LCD_RST_GPIO)
#define LCD_RST_LOW() gpio_clear(LCD_RST_GPIO)

namespace {

constexpr uint8_t ST7567_DISPLAY_OFF = 0xAE;
constexpr uint8_t ST7567_DISPLAY_ON = 0xAF;
constexpr uint8_t ST7567_ALL_PIXELS_NORMAL = 0xA4;
constexpr uint8_t ST7567_ALL_PIXELS_ON = 0xA5;
constexpr uint8_t ST7567_NORMAL_DISPLAY = 0xA6;
constexpr uint8_t ST7567_INVERSE_DISPLAY = 0xA7;
constexpr uint8_t ST7567_BIAS_1_9 = 0xA2;
constexpr uint8_t ST7567_START_LINE_0 = 0x40;
constexpr uint8_t ST7567_REGULATOR_RATIO_5_5 = 0x25;
constexpr uint8_t ST7567_SET_EV = 0x81;
constexpr uint8_t ST7567_DEFAULT_EV = 40;
constexpr uint8_t ST7567_POWER_BOOSTER = 0x2C;
constexpr uint8_t ST7567_POWER_REGULATOR = 0x2E;
constexpr uint8_t ST7567_POWER_FOLLOWER = 0x2F;
constexpr uint8_t ST7567_SET_BOOSTER = 0xF8;
constexpr uint8_t ST7567_BOOSTER_4X = 0x00;
constexpr uint8_t ST7567_NOP = 0xE3;
constexpr uint8_t ST7567_CONTRAST_OFFSET = 20;
constexpr uint8_t ST7567_EV_MAX = 63;
constexpr uint8_t ST7567_PAGE_COUNT = LCD_H / 8;
constexpr uint32_t LCD_DMA_TIMEOUT_MS = 20;

static_assert(LCD_W == 128 && LCD_H == 64 && LCD_DEPTH == 1,
              "The HELM ST7567 driver requires the 128x64 1-bit framebuffer");
static_assert(DISPLAY_BUFFER_SIZE == LCD_W * ST7567_PAGE_COUNT,
              "Unexpected EdgeTX framebuffer layout");
static_assert(LCD_COLUMN_OFFSET + LCD_W <= 132,
              "The ST7567 has only 132 columns");

volatile bool lcdBusy;
volatile uint8_t lcdPage;
volatile uint32_t lcdRefreshCount;
volatile uint32_t lcdDmaErrorCount;
bool lcdDisplayOn;
pixel_t lcdDmaBuffer[DISPLAY_BUFFER_SIZE] __DMA;

void lcdWaitForSpiComplete()
{
  while ((LCD_SPI->SR & SPI_SR_TXC) == 0) {
  }
}

void lcdSerialHoldTime()
{
  // ST7567 requires CS and A0 to remain stable for at least 40 ns after the
  // final rising clock edge. Sixteen CPU cycles are 40 ns at 400 MHz.
  for (uint8_t i = 0; i < 16; ++i) {
    __NOP();
  }
}

void lcdWriteCommand(uint8_t command)
{
  LCD_A0_LOW();
  LCD_NCS_LOW();

  lcdWaitForSpiComplete();
  *reinterpret_cast<volatile uint8_t *>(&LCD_SPI->TXDR) = command;
  LCD_SPI->CR1 |= SPI_CR1_CSTART;
  lcdWaitForSpiComplete();
  lcdSerialHoldTime();

  LCD_NCS_HIGH();
  LCD_A0_HIGH();
}

void lcdConfigureController()
{
  // Powertip PE12864WRF-055-H-Q_005 recommended initialization values.
  lcdWriteCommand(ST7567_DISPLAY_OFF);
  lcdWriteCommand(ST7567_BIAS_1_9);
  lcdWriteCommand(LCD_SEG_DIRECTION_COMMAND);
  lcdWriteCommand(LCD_COM_DIRECTION_COMMAND);
  lcdWriteCommand(ST7567_NORMAL_DISPLAY);
  lcdWriteCommand(ST7567_ALL_PIXELS_NORMAL);
  lcdWriteCommand(ST7567_START_LINE_0);
  lcdWriteCommand(ST7567_REGULATOR_RATIO_5_5);
  lcdWriteCommand(ST7567_SET_EV);
  lcdWriteCommand(ST7567_DEFAULT_EV);

  // Enable the booster, regulator, and voltage follower in stages.
  lcdWriteCommand(ST7567_POWER_BOOSTER);
  lcdWriteCommand(ST7567_NOP);
  lcdWriteCommand(ST7567_POWER_REGULATOR);
  lcdWriteCommand(ST7567_NOP);
  lcdWriteCommand(ST7567_POWER_FOLLOWER);
  lcdWriteCommand(ST7567_NOP);
  lcdWriteCommand(ST7567_SET_BOOSTER);
  lcdWriteCommand(ST7567_BOOSTER_4X);
}

void lcdStartPage(uint8_t page)
{
  constexpr uint8_t column = LCD_COLUMN_OFFSET;

  lcdWriteCommand(0xB0 | page);
  lcdWriteCommand(0x10 | (column >> 4));
  lcdWriteCommand(column & 0x0F);

  LCD_NCS_LOW();
  LCD_A0_HIGH();

  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN;
  while ((LCD_DMA_Stream->CR & DMA_SxCR_EN) != 0) {
  }

  LCD_DMA->HIFCR = LCD_DMA_FLAGS;
  LCD_DMA_Stream->M0AR =
      reinterpret_cast<uint32_t>(&lcdDmaBuffer[page * LCD_W]);
  LCD_DMA_Stream->NDTR = LCD_W;
  LCD_DMA_Stream->CR |=
      DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_EN;
  LCD_DMA_Stream->FCR |= DMA_SxFCR_FEIE;
  LCD_SPI->CFG1 |= SPI_CFG1_TXDMAEN;
}

void lcdFinishDmaTransfer()
{
  LCD_DMA_Stream->CR &=
      ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_EN);
  LCD_DMA_Stream->FCR &= ~DMA_SxFCR_FEIE;
  while ((LCD_DMA_Stream->CR & DMA_SxCR_EN) != 0) {
  }
  LCD_SPI->CFG1 &= ~SPI_CFG1_TXDMAEN;
}

void lcdRecoverSpi()
{
  LCD_SPI->CR1 &= ~SPI_CR1_SPE;
  LCD_SPI->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
                  SPI_IFCR_OVRC | SPI_IFCR_CRCEC | SPI_IFCR_TIFREC |
                  SPI_IFCR_MODFC;
  LCD_SPI->CR1 |= SPI_CR1_SPE;
}

void lcdAbortDmaTransfer()
{
  lcdFinishDmaTransfer();
  LCD_DMA->HIFCR = LCD_DMA_FLAGS;
  NVIC_ClearPendingIRQ(LCD_DMA_Stream_IRQn);
  LCD_NCS_HIGH();
  lcdRecoverSpi();
  ++lcdDmaErrorCount;
  lcdBusy = false;
}

void lcdHardwareInit()
{
  // Establish defined levels before connecting SCK and MOSI to SPI4.
  gpio_init(LCD_NCS_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  LCD_NCS_HIGH();
  gpio_init(LCD_A0_GPIO, GPIO_OUT, GPIO_PIN_SPEED_HIGH);
  LCD_A0_LOW();
  gpio_init(LCD_RST_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  LCD_RST_LOW();
  gpio_init(LCD_CLK_GPIO, GPIO_OUT, GPIO_PIN_SPEED_HIGH);
  gpio_clear(LCD_CLK_GPIO);
  gpio_init(LCD_MOSI_GPIO, GPIO_OUT, GPIO_PIN_SPEED_HIGH);
  gpio_clear(LCD_MOSI_GPIO);

  stm32_spi_enable_clock(LCD_SPI);
  LCD_SPI->CR1 &= ~SPI_CR1_SPE;
  LCD_SPI->CR1 = SPI_CR1_SSI | SPI_CR1_HDDIR;
  LCD_SPI->CR2 = 0;
  LCD_SPI->CFG1 = (7U << SPI_CFG1_DSIZE_Pos) | LCD_SPI_PRESCALER;
  LCD_SPI->CFG2 = SPI_CFG2_SSM | SPI_CFG2_MASTER |
                  (3U << SPI_CFG2_COMM_Pos);  // TX-only, SPI mode 0
  LCD_SPI->CR1 |= SPI_CR1_SPE;

  gpio_init_af(LCD_CLK_GPIO, LCD_GPIO_AF, GPIO_PIN_SPEED_VERY_HIGH);
  gpio_init_af(LCD_MOSI_GPIO, LCD_GPIO_AF, GPIO_PIN_SPEED_VERY_HIGH);

  stm32_dma_enable_clock(LCD_DMA);
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN;
  while ((LCD_DMA_Stream->CR & DMA_SxCR_EN) != 0) {
  }
  LCD_DMA->HIFCR = LCD_DMA_FLAGS;
  LCD_DMA_Stream->CR = DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  LCD_DMA_Stream->PAR = reinterpret_cast<uint32_t>(&LCD_SPI->TXDR);
  LCD_DMA_Stream->M0AR = reinterpret_cast<uint32_t>(lcdDmaBuffer);
  LCD_DMA_Stream->NDTR = LCD_W;
  LCD_DMA_Stream->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;
  LL_DMA_SetPeriphRequest(LCD_DMA, LCD_DMA_Stream_Num,
                          LL_DMAMUX1_REQ_SPI4_TX);

  NVIC_SetPriority(LCD_DMA_Stream_IRQn, 7);
  NVIC_EnableIRQ(LCD_DMA_Stream_IRQn);
}

}  // namespace

bool lcdInitFinished = false;

void lcdRefreshWait()
{
  const uint32_t started = timersGetMsTick();
  while (lcdBusy) {
    if (timersGetMsTick() - started >= LCD_DMA_TIMEOUT_MS) {
      const uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (lcdBusy) {
        lcdAbortDmaTransfer();
      }
      if (primask == 0) {
        __enable_irq();
      }
      break;
    }
  }
}

void lcdRefresh(bool wait)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  lcdRefreshWait();
  std::memcpy(lcdDmaBuffer, displayBuf, DISPLAY_BUFFER_SIZE);
  SCB_CleanDCache_by_Addr(lcdDmaBuffer, DISPLAY_BUFFER_SIZE);

  lcdPage = 0;
  lcdBusy = true;
  lcdStartPage(lcdPage);

  if (wait) {
    lcdRefreshWait();
  }
}

extern "C" void LCD_DMA_Stream_IRQHandler()
{
  const uint32_t status = LCD_DMA->HISR;
  lcdFinishDmaTransfer();
  LCD_DMA->HIFCR = LCD_DMA_FLAGS;

  if ((status & LCD_DMA_STATUS_ERRORS) != 0) {
    lcdAbortDmaTransfer();
    return;
  }

  if ((status & LCD_DMA_STATUS_COMPLETE) == 0) {
    lcdAbortDmaTransfer();
    return;
  }

  // DMA completion precedes the last bits leaving the SPI FIFO.
  lcdWaitForSpiComplete();
  lcdSerialHoldTime();
  LCD_NCS_HIGH();

  if (++lcdPage < ST7567_PAGE_COUNT) {
    lcdStartPage(lcdPage);
    return;
  }

  if (!lcdDisplayOn) {
    // This order also exits the compound AE/A5 power-save state.
    lcdWriteCommand(ST7567_DISPLAY_ON);
    lcdWriteCommand(ST7567_ALL_PIXELS_NORMAL);
    lcdDisplayOn = true;
  }

  ++lcdRefreshCount;
  lcdBusy = false;
  lcdFlushed();
}

void lcdInit()
{
  if (lcdInitFinished) return;

  lcdHardwareInit();

  // Hardware reset is mandatory for ST7567. All function and power-control
  // commands below are issued within 5 ms of releasing reset.
  LCD_RST_LOW();
  delay_ms(1);
  LCD_RST_HIGH();
  delay_ms(1);
  lcdConfigureController();

  // Let the generated LCD supply settle, then initialize all visible DDRAM
  // before allowing the controller to drive the glass.
  delay_ms(20);
  lcdInitFinished = true;
  lcdClear();
  lcdRefresh(true);
}

void lcdInitFinish()
{
  if (!lcdInitFinished) {
    lcdInit();
  }
}

void lcdOff()
{
  if (!lcdInitFinished) return;

  lcdRefreshWait();
  lcdWriteCommand(ST7567_DISPLAY_OFF);
  lcdWriteCommand(ST7567_ALL_PIXELS_ON);
  lcdDisplayOn = false;
}

void lcdSetRefVolt(uint8_t value)
{
  lcdInitFinish();
  lcdRefreshWait();

  uint16_t ev = static_cast<uint16_t>(value) + ST7567_CONTRAST_OFFSET;
  if (ev > ST7567_EV_MAX) ev = ST7567_EV_MAX;

  lcdWriteCommand(ST7567_SET_EV);
  lcdWriteCommand(static_cast<uint8_t>(ev));
}

void lcdSetInvert(bool invert)
{
  lcdInitFinish();
  lcdRefreshWait();
  lcdWriteCommand(invert ? ST7567_INVERSE_DISPLAY : ST7567_NORMAL_DISPLAY);
}

uint32_t lcdGetRefreshCount()
{
  return lcdRefreshCount;
}

uint32_t lcdGetDmaErrorCount()
{
  return lcdDmaErrorCount;
}
