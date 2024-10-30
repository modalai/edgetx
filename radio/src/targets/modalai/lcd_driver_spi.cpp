/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_dma.h"

#include "board.h"
#include "debug.h"
#include "lcd.h"

#include "hal/abnormal_reboot.h"
#include "timers_driver.h"

#if !defined(BOOT)
  #include "edgetx.h"
#endif

#if defined(OLED_SCREEN)
  #define LCD_CONTRAST_OFFSET            0
#elif defined(MANUFACTURER_RADIOMASTER)
  #define LCD_CONTRAST_OFFSET            -10
#else
  #define LCD_CONTRAST_OFFSET            160
#endif
#define RESET_WAIT_DELAY_MS            300 // Wait time after LCD reset before first command
#define WAIT_FOR_DMA_END()             do { lcd_sr = LCD_SPI->SR; lcd_dma_sr = LCD_DMA->ISR; } while (lcd_busy) // TODO: Fix this

#define LCD_NCS_HIGH()  gpio_set(LCD_NCS_GPIO)
#define LCD_NCS_LOW()   gpio_clear(LCD_NCS_GPIO)

#define LCD_A0_HIGH()   gpio_set(LCD_A0_GPIO)
#define LCD_A0_LOW()    gpio_clear(LCD_A0_GPIO)

#define LCD_RST_HIGH()  gpio_set(LCD_RST_GPIO)
#define LCD_RST_LOW()   gpio_clear(LCD_RST_GPIO)

uint32_t lcd_sr;
uint32_t lcd_dma_sr;

bool lcdInitFinished = false;
void lcdInitFinish();

void lcdWriteCommand(uint8_t byte)
{
  LCD_A0_LOW();
  LCD_NCS_LOW();
  while ((LCD_SPI->SR & SPI_SR_TXC) == 0) {
    // Wait
  }
  //(void)LCD_SPI->RXDR; // Clear receive
  *((volatile uint8_t *)&LCD_SPI->TXDR) = byte; // Must limit to 8-bit bus transaction
  LCD_SPI->CR1 |= SPI_CR1_CSTART;
  while ((LCD_SPI->SR & SPI_SR_TXC) == 0) {
    // Wait
  }
  LCD_NCS_HIGH();
}

void lcdHardwareInit()
{
  // stm32_spi_enable_clock(LCD_SPI);
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
  gpio_init_af(LCD_MOSI_GPIO, LCD_GPIO_AF, GPIO_PIN_SPEED_VERY_HIGH);
  gpio_init_af(LCD_CLK_GPIO, GPIO_AF8, GPIO_PIN_SPEED_VERY_HIGH);
  gpio_init(LCD_NCS_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  gpio_init(LCD_RST_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  gpio_init(LCD_A0_GPIO, GPIO_OUT, GPIO_PIN_SPEED_HIGH);

  LCD_NCS_HIGH();

  // APB1 clock / 2 = 133nS per clock
  LCD_SPI->CR1 &= ~(SPI_CR1_SPE);
  LCD_SPI->CR1 = 0; // Clear any mode error
  LCD_SPI->CR1 = SPI_CR1_SSI | SPI_CR1_HDDIR;
  LCD_SPI->CR2 = 0;
  LCD_SPI->CFG1 = 0x00070007;
  LCD_SPI->CFG1 |= LCD_SPI_PRESCALER;
  LCD_SPI->CFG2 = SPI_CFG2_CPHA | SPI_CFG2_CPOL | SPI_CFG2_SSM | SPI_CFG2_MASTER | (0x3 << SPI_CFG2_COMM_Pos);
  // LCD_SPI->CFG2 = SPI_CFG2_CPHA | SPI_CFG2_CPOL | SPI_CFG2_SSM | SPI_CFG2_MASTER;
  LCD_SPI->CR1 |= SPI_CR1_SPE;


  stm32_bdma_enable_clock(LCD_DMA);
  LCD_DMA_Stream->CCR = 0x1092;
  // LCD_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA
  // DMAMUX configuration
  //LCD_DMAMUX->CCR &= ~(DMAMUX_CxCR_DMAREQ_ID); // Clear request
  // LCD_DMAMUX->CCR |=  ( 12U ); // SPI6 TX DMA (unsure if this should be -1 or not since reference manual is 1-indexed)
  LL_BDMA_SetPeriphRequest(LCD_DMA, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_TX);
  // Use the LL library instead to ensure the request is set correctly

  // LCD_DMA->HIFCR = LCD_DMA_FLAGS; // Write ones to clear bits
  // LCD_DMA_Stream->CR =  DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  LCD_DMA_Stream->CPAR = (uint32_t)&LCD_SPI->TXDR;
#if LCD_W == 128
  LCD_DMA_Stream->CNDTR = LCD_W;
#else
  LCD_DMA_Stream->CM0AR = (uint32_t)displayBuf;
  LCD_DMA_Stream->CNDTR = LCD_W*LCD_H/8*4;
#endif
  LCD_DMA->IFCR |= LCD_DMA_FLAGS; // DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;

  NVIC_SetPriority(LCD_DMA_Stream_IRQn, 7);
  NVIC_EnableIRQ(LCD_DMA_Stream_IRQn);
}


void lcdStart()
{
//  #if defined(RADIO_MODAL)
    // lcdWriteCommand(0xa0); // Set seg
    // lcdWriteCommand(0xaF); // Turn on display
    // lcdWriteCommand(0xA5); // Turn on whole display
#if defined(LCD_VERTICAL_INVERT)
  // T12 and TX12 have the screen inverted.
    lcdWriteCommand(0xe2); // (14) Soft reset
#if defined(LCD_HORIZONTAL_INVERT)
    lcdWriteCommand(0xa1); // Set seg
#else 
    lcdWriteCommand(0xa0); // Set seg
#endif
    lcdWriteCommand(0xc8); // Set com
    lcdWriteCommand(0xf8); // Set booster
    lcdWriteCommand(0x01); // 5x (0x00 is 4x according to datasheet?)
    lcdWriteCommand(0xa3); // Set bias=1/6
    lcdWriteCommand(0x22); // Set internal rb/ra=5.0
    lcdWriteCommand(0x2f); // All built-in power circuits on
    // lcdWriteCommand(0x24); // Power control set
    lcdWriteCommand(0x81); // Set contrast
    lcdWriteCommand(0x0A); // Set Vop
    lcdWriteCommand(0xa6); // Set display mode
#else
    lcdWriteCommand(0xe2); // (14) Soft reset
    lcdWriteCommand(0xa1); // Set seg
    lcdWriteCommand(0xc0); // Set com
    lcdWriteCommand(0xf8); // Set booster
    lcdWriteCommand(0x00); // 5x
    lcdWriteCommand(0xa3); // Set bias=1/6
    lcdWriteCommand(0x22); // Set internal rb/ra=5.0
    lcdWriteCommand(0x2f); // All built-in power circuits on
    lcdWriteCommand(0x81); // Set contrast
    lcdWriteCommand(0x36); // Set Vop
    lcdWriteCommand(0xa6); // Set display mode
#endif
#if defined(BOOT)
  lcdSetRefVolt(LCD_CONTRAST_DEFAULT);
#else
  lcdSetRefVolt(LCD_CONTRAST_DEFAULT);
#endif
}

volatile bool lcd_busy;

void lcdRefreshWait()
{
  WAIT_FOR_DMA_END();
}

void lcdRefresh(bool wait)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  SCB_CleanDCache();

  uint8_t * p = displayBuf;
  uint32_t cfg2 = LCD_SPI->CFG2;
#if defined(LCD_W_OFFSET)
  lcdWriteCommand(LCD_W_OFFSET);
#endif
  for (uint8_t y=0; y < 8; y++, p+=LCD_W) {

    __disable_irq();

    lcdWriteCommand(0x10); // Column addr 0
    lcdWriteCommand(0xB0 | y); // Page addr y
#if !defined(LCD_VERTICAL_INVERT)
    lcdWriteCommand(0x04);
#endif

    LCD_NCS_LOW();
    LCD_A0_HIGH();

    lcd_busy = true;
    LCD_DMA_Stream->CCR &= ~BDMA_CCR_EN; // Disable DMA
    LCD_DMA->IFCR |= LCD_DMA_FLAGS; // Write ones to clear bits
    LCD_DMA_Stream->CM0AR = (uint32_t)p;
#if LCD_W == 128
  LCD_DMA_Stream->CNDTR = LCD_W;
#else
  LCD_DMA_Stream->CNDTR = LCD_W*LCD_H/8*4;
#endif
    LCD_SPI->CFG1 |= SPI_CFG1_TXDMAEN;
    LCD_DMA_Stream->CCR |=  BDMA_CCR_TCIE;
    LCD_DMA_Stream->CCR |= BDMA_CCR_EN; // Enable DMA & TC interrupts

    __enable_irq(); // Don't preempt this thread until the DMA transaction has started

    WAIT_FOR_DMA_END();

    __disable_irq();

    LCD_NCS_HIGH();
    LCD_A0_HIGH();
  }

  __enable_irq();
}

extern "C" void LCD_DMA_Stream_IRQHandler()
{
  DEBUG_INTERRUPT(INT_LCD);

  LCD_DMA_Stream->CCR &= ~BDMA_CCR_TCIE; // Stop interrupt
  LCD_DMA->IFCR |= LCD_DMA_FLAG_INT; // Clear interrupt flag
  LCD_SPI->CFG1 &= ~SPI_CFG1_TXDMAEN;
  LCD_DMA_Stream->CCR &= ~BDMA_CCR_EN; // Disable DMA

  while ((LCD_SPI->SR & SPI_SR_TXC) == 0) {
    /* Wait for SPI to finish sending data
    The DMA TX End interrupt comes two bytes before the end of SPI transmission,
    therefore we have to wait here.
    */
  }
  LCD_NCS_HIGH();
  lcd_busy = false;
}

/*
  Proper method for turning of LCD module. It must be used,
  otherwise we might damage LCD crystals in the long run!
*/
void lcdOff()
{
  WAIT_FOR_DMA_END();

  /*
  LCD Sleep mode is also good for draining capacitors and enables us
  to re-init LCD without any delay
  */
  lcdWriteCommand(0xAE); // LCD sleep
  delay_ms(3); // Wait for caps to drain
}

void lcdReset()
{
  LCD_NCS_HIGH();
  LCD_RST_LOW();
#if LCD_W == 128
  delay_ms(150);
#else
  delay_ms(1); // Only 3 us needed according to data-sheet, we use 1 ms
#endif
  LCD_RST_HIGH();
}

/*
  Starts LCD initialization routine. It should be called as
  soon as possible after the reset because LCD takes a lot of
  time to properly power-on.

  Make sure that delay_ms() is functional before calling this function!
*/
void lcdInit()
{
  lcdHardwareInit();

  if (IS_LCD_RESET_NEEDED()) {
    lcdReset();
  }
}

/*
  Finishes LCD initialization. It is called auto-magically when first LCD command is
  issued by the other parts of the code.
*/

#if defined(RADIO_X9DP2019) || defined(RADIO_X7ACCESS)
  #define LCD_DELAY_NEEDED() true
#else
  #define LCD_DELAY_NEEDED() (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE())
#endif

void lcdInitFinish()
{
  lcdInitFinished = true;

  /*
    LCD needs longer time to initialize in low temperatures. The data-sheet
    mentions a time of at least 150 ms. The delay of 1300 ms was obtained
    experimentally. It was tested down to -10 deg Celsius.

    The longer initialization time seems to only be needed for regular Taranis,
    the Taranis Plus (9XE) has been tested to work without any problems at -18 deg Celsius.
    Therefore the delay for T+ is lower.

    If radio is reset by watchdog or boot-loader the wait is skipped, but the LCD
    is initialized in any case.

    This initialization is needed in case the user moved power switch to OFF and
    then immediately to ON position, because lcdOff() was called. In any case the LCD
    initialization (without reset) is also recommended by the data sheet.
  */

  if (LCD_DELAY_NEEDED()) {
    uint32_t end = timersGetMsTick() + RESET_WAIT_DELAY_MS;
    while (timersGetMsTick() < end);
  }

  lcdStart();
  lcdWriteCommand(0xAF); // dc2=1, IC into exit SLEEP MODE, dc3=1 gray=ON, dc4=1 Green Enhanc mode disabled
  delay_ms(20); // needed for internal DC-DC converter startup
}

void lcdSetRefVolt(uint8_t val)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  lcdWriteCommand(0x81); // Set Vop
  lcdWriteCommand(val+LCD_CONTRAST_OFFSET); // 0-255
}

void lcdSetInvert(bool invert)
{
   lcdWriteCommand(invert ? 0xA7 : 0xA6);
}
