/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#include "rtt_key_injection.h"

#if defined(HELM_HOST_KEY_INJECTION) && defined(DEBUG_SEGGER_RTT)

#include "hal/key_driver.h"

#if !defined(BOOT)
#include "thirdparty/Segger/SEGGER/SEGGER_RTT.h"
#include "timers_driver.h"
#endif

namespace {

constexpr uint8_t KEY_MASK_DIGITS = 8;
constexpr uint8_t FRAME_SIZE = KEY_MASK_DIGITS + 2;

volatile uint32_t hostKeyMask = 0;

#if !defined(BOOT)
constexpr tmr10ms_t HOST_KEY_TIMEOUT = 30;
uint32_t receivedKeyMask = 0;
tmr10ms_t lastFrameTime = 0;
uint8_t framePosition = FRAME_SIZE;
bool frameActive = false;

int8_t hexValue(uint8_t value)
{
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

void parseByte(uint8_t value)
{
  if (value == 'K') {
    receivedKeyMask = 0;
    framePosition = 1;
    return;
  }

  if (framePosition >= 1 && framePosition <= KEY_MASK_DIGITS) {
    int8_t digit = hexValue(value);
    if (digit < 0) {
      framePosition = FRAME_SIZE;
      return;
    }

    receivedKeyMask = (receivedKeyMask << 4) | digit;
    framePosition++;
    return;
  }

  if (framePosition == KEY_MASK_DIGITS + 1 &&
      (value == '\n' || value == '\r')) {
    hostKeyMask = receivedKeyMask;
    lastFrameTime = get_tmr10ms();
    frameActive = true;
  }
  framePosition = FRAME_SIZE;
}
#endif

}  // namespace

uint32_t hostKeyInjectionRead()
{
  return hostKeyMask;
}

void rttKeyInjectionPoll()
{
#if !defined(BOOT)
  uint8_t data[16];
  unsigned count = SEGGER_RTT_Read(0, data, sizeof(data));
  for (unsigned index = 0; index < count; index++) {
    parseByte(data[index]);
  }

  if (frameActive &&
      (tmr10ms_t)(get_tmr10ms() - lastFrameTime) >= HOST_KEY_TIMEOUT) {
    hostKeyMask = 0;
    frameActive = false;
  }
#endif
}

#endif
