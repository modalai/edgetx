/*
 * Headless M0196 LCD contract.
 *
 * This intentionally keeps the EdgeTX 128x64 GUI and bootloader call paths
 * alive until the Helm panel driver is available.
 */

#include "board.h"

bool lcdInitFinished = false;

void lcdInit()
{
  lcdInitFinished = true;
}

void lcdInitFinish()
{
  lcdInitFinished = true;
}

void lcdRefresh(bool)
{
  lcdFlushed();
}

void lcdOff()
{
}

void lcdSetRefVolt(uint8_t)
{
}

void lcdSetInvert(bool)
{
}

