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

#include "edgetx.h"

#if defined(PCBMODALAI)

enum {
  LCD_TUNE_GPIO_SPEED = 0,
  LCD_TUNE_SPI_PRESCALER,
  LCD_TUNE_RESISTOR_RATIO,
  LCD_TUNE_BIAS,
  LCD_TUNE_BOOSTER,
  LCD_TUNE_VOP,
  LCD_TUNE_COUNT
};

static uint8_t lcdTuneGpioSpeed = 3;      // 0=LOW, 1=MED, 2=HIGH, 3=VHIGH (current)
static uint8_t lcdTuneSpiPrescaler = 1;   // MBR value: 1=/4(25MHz), 2=/8(12.5MHz), 3=/16, 4=/32
static uint8_t lcdTuneResistorRatio = 4;  // 0-7, actual init ends at 0x24 = ratio 5.0
static uint8_t lcdTuneBias = 1;           // 0=1/9(0xa2), 1=1/7(0xa3)
static uint8_t lcdTuneBooster = 0;        // 0-3 (5x-8x)
static uint8_t lcdTuneVop = 10;           // 0-63, default 0x0A

static const char* const gpioSpeedLabels[] = { "LOW", "MED", "HIGH", "VHIGH" };
static const char* const biasLabels[] = { "1/9", "1/7" };
static const uint8_t boosterMult[] = { 5, 6, 7, 8 };
static const char* const spiClockLabels[] = { "/2 50M", "/4 25M", "/8 12M", "/16 6M", "/32 3M" };

void menuRadioLcdTuning(event_t event)
{
  SIMPLE_SUBMENU("LCD Tuning", LCD_TUNE_COUNT);

  if (event == EVT_ENTRY) {
    lcdTuneVop = g_eeGeneral.contrast;
  }

  uint8_t sub = menuVerticalPosition - HEADER_LINE;

  for (uint8_t i = 0; i < LCD_TUNE_COUNT && i < NUM_BODY_LINES; i++) {
    coord_t y = MENU_HEADER_HEIGHT + 1 + i * FH;
    uint8_t k = i + menuVerticalOffset;
    LcdFlags attr = (sub == k) ? INVERS : 0;

    switch (k) {
      case LCD_TUNE_GPIO_SPEED:
        lcdDrawTextAlignedLeft(y, "GPIO Spd");
        lcdDrawText(LCD_W - 6 * FW, y, gpioSpeedLabels[lcdTuneGpioSpeed], attr);
        if (attr) {
          lcdTuneGpioSpeed = checkIncDec(event, lcdTuneGpioSpeed, 0, 3, 0);
          if (checkIncDec_Ret) {
            lcdSetGpioSpeed(lcdTuneGpioSpeed);
          }
        }
        break;

      case LCD_TUNE_SPI_PRESCALER:
        lcdDrawTextAlignedLeft(y, "SPI Clk");
        lcdDrawText(LCD_W - 7 * FW, y, spiClockLabels[lcdTuneSpiPrescaler], attr);
        if (attr) {
          lcdTuneSpiPrescaler = checkIncDec(event, lcdTuneSpiPrescaler, 1, 4, 0);
          if (checkIncDec_Ret) {
            lcdSetSpiPrescaler(lcdTuneSpiPrescaler);
          }
        }
        break;

      case LCD_TUNE_RESISTOR_RATIO:
        lcdDrawTextAlignedLeft(y, "Ratio Rb/Ra");
        lcdDrawNumber(LCD_W - 3 * FW, y, 30 + lcdTuneResistorRatio * 5, PREC1 | attr);
        if (attr) {
          lcdTuneResistorRatio = checkIncDec(event, lcdTuneResistorRatio, 0, 7, 0);
          if (checkIncDec_Ret) {
            lcdSetResistorRatio(lcdTuneResistorRatio);
          }
        }
        break;

      case LCD_TUNE_BIAS:
        lcdDrawTextAlignedLeft(y, "Bias");
        lcdDrawText(LCD_W - 4 * FW, y, biasLabels[lcdTuneBias], attr);
        if (attr) {
          lcdTuneBias = checkIncDec(event, lcdTuneBias, 0, 1, 0);
          if (checkIncDec_Ret) {
            lcdSetBias(lcdTuneBias);
          }
        }
        break;

      case LCD_TUNE_BOOSTER:
        lcdDrawTextAlignedLeft(y, "Booster");
        lcdDrawNumber(LCD_W - 3 * FW, y, boosterMult[lcdTuneBooster], attr);
        lcdDrawChar(lcdNextPos, y, 'x');
        if (attr) {
          lcdTuneBooster = checkIncDec(event, lcdTuneBooster, 0, 3, 0);
          if (checkIncDec_Ret) {
            lcdSetBoosterRatio(lcdTuneBooster);
          }
        }
        break;

      case LCD_TUNE_VOP:
        lcdDrawTextAlignedLeft(y, "Vop");
        lcdDrawNumber(LCD_W - 3 * FW, y, lcdTuneVop, attr);
        if (attr) {
          lcdTuneVop = checkIncDec(event, lcdTuneVop, 0, 63, 0);
          if (checkIncDec_Ret) {
            lcdSetVop(lcdTuneVop);
            g_eeGeneral.contrast = lcdTuneVop;
            storageDirty(EE_GENERAL);
          }
        }
        break;
    }
  }
}

#endif // PCBMODALAI
