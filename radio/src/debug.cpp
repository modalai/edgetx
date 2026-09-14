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
#include "stamp.h"
#include <stdarg.h>

#if defined(DEBUG_SEGGER_RTT) && !defined(BOOT)
#include "hal/adc_driver.h"
#include "hal/key_driver.h"
#include "hal/module_port.h"
#include "hal/switch_driver.h"
#include "mixes.h"
#include "pulses/pulses.h"
#include "switches.h"

#include <string.h>
#endif

#if defined(SIMU)
traceCallbackFunc traceCallback = 0;
#endif

#if defined(DEBUG_SEGGER_RTT) && !defined(BOOT)

static_assert(MAX_KEYS <= DEBUG_MONITOR_MAX_KEYS,
              "Debug monitor key capacity is too small");
static_assert(MAX_TRIMS * 2 <= DEBUG_MONITOR_MAX_TRIMS,
              "Debug monitor trim capacity is too small");
static_assert(MAX_SWITCHES <= DEBUG_MONITOR_MAX_SWITCHES,
              "Debug monitor switch capacity is too small");
static_assert(MAX_ANALOG_INPUTS <= DEBUG_MONITOR_MAX_ANALOGS,
              "Debug monitor analog capacity is too small");
static_assert(MAX_INPUTS <= DEBUG_MONITOR_MAX_INPUTS,
              "Debug monitor input capacity is too small");
static_assert(MAX_OUTPUT_CHANNELS <= DEBUG_MONITOR_MAX_CHANNELS,
              "Debug monitor channel capacity is too small");
static_assert(NUM_MODULES <= DEBUG_MONITOR_MAX_MODULES,
              "Debug monitor module capacity is too small");
static_assert(sizeof(DebugMonitorModulePort) == 8,
              "Debug monitor port ABI changed");
static_assert(sizeof(DebugMonitorModule) == 20,
              "Debug monitor module ABI changed");
static_assert(offsetof(DebugMonitorSnapshot, sequence) == 8,
              "Debug monitor sequence offset changed");
static_assert(sizeof(DebugMonitorSnapshot) == 1428,
              "Debug monitor snapshot ABI changed");

extern "C" {
DebugMonitorSnapshot debugMonitorSnapshot = {};
}

static void copyDebugMonitorLabel(char *destination, const char *source,
                                  size_t sourceCapacity =
                                      DEBUG_MONITOR_LABEL_LENGTH - 1)
{
  memset(destination, 0, DEBUG_MONITOR_LABEL_LENGTH);
  if (!source) return;

  const size_t capacity =
      min(sourceCapacity, (size_t)DEBUG_MONITOR_LABEL_LENGTH - 1);
  size_t length = 0;
  while (length < capacity && source[length]) ++length;
  memcpy(destination, source, length);
}

static void captureDebugMonitorPort(DebugMonitorModulePort &destination,
                                    const etx_module_driver_t &source,
                                    uint8_t direction)
{
  memset(&destination, 0, sizeof(destination));
  if (!source.port) return;

  destination.active = 1;
  destination.type = source.port->type;
  destination.port = source.port->port;
  destination.direction = direction;

  if (source.port->type == ETX_MOD_TYPE_SERIAL) {
    auto driver = modulePortGetSerialDrv(source);
    if (driver && driver->getBaudrate && source.ctx) {
      destination.baudrate = driver->getBaudrate(source.ctx);
    }
  }
}

static uint32_t getDebugMonitorActiveInputs()
{
  uint32_t active = 0;
  for (uint8_t i = 0; i < MAX_EXPOS; ++i) {
    auto input = expoAddress(i);
    if (!EXPO_VALID(input)) break;
    if (input->chn < DEBUG_MONITOR_MAX_INPUTS) {
      active |= (1u << input->chn);
    }
  }
  return active;
}

static uint32_t getDebugMonitorActiveChannels()
{
  uint32_t active = 0;
  for (uint8_t i = 0; i < MAX_MIXERS; ++i) {
    auto mix = mixAddress(i);
    if (!mix->srcRaw) break;
    if (mix->destCh < DEBUG_MONITOR_MAX_CHANNELS) {
      active |= (1u << mix->destCh);
    }
  }
  return active;
}

void debugMonitorCapture()
{
  static tmr10ms_t lastCapture = (tmr10ms_t)-10;
  const tmr10ms_t now = get_tmr10ms();
  if ((tmr10ms_t)(now - lastCapture) < 10) return;
  lastCapture = now;

  uint32_t sequence = debugMonitorSnapshot.sequence + 1;
  if (!(sequence & 1u)) ++sequence;
  debugMonitorSnapshot.sequence = sequence;
  __DMB();

  debugMonitorSnapshot.magic = DEBUG_MONITOR_MAGIC;
  debugMonitorSnapshot.version = DEBUG_MONITOR_VERSION;
  debugMonitorSnapshot.size = sizeof(debugMonitorSnapshot);
  debugMonitorSnapshot.tick10ms = now;
  debugMonitorSnapshot.keysSupported = keysGetSupported();
  debugMonitorSnapshot.keysPressed = readKeys();
  debugMonitorSnapshot.trimsPressed = readTrims();
  debugMonitorSnapshot.activeInputs = getDebugMonitorActiveInputs();
  debugMonitorSnapshot.activeChannels = getDebugMonitorActiveChannels();
  debugMonitorSnapshot.functionSwitches = 0;
  debugMonitorSnapshot.keyCount = keysGetMaxKeys();
  debugMonitorSnapshot.trimCount = keysGetMaxTrims() * 2;
  debugMonitorSnapshot.switchCount = switchGetMaxSwitches();
  debugMonitorSnapshot.mainAnalogCount = adcGetMaxInputs(ADC_INPUT_MAIN);
  debugMonitorSnapshot.flexAnalogCount = adcGetMaxInputs(ADC_INPUT_FLEX);
  debugMonitorSnapshot.batteryAnalogCount = adcGetMaxInputs(ADC_INPUT_VBAT);
  debugMonitorSnapshot.rtcBatteryAnalogCount =
      adcGetMaxInputs(ADC_INPUT_RTC_BAT);
  debugMonitorSnapshot.analogCount = adcGetMaxInputs(ADC_INPUT_ALL);
  debugMonitorSnapshot.moduleCount = NUM_MODULES;
  memset(debugMonitorSnapshot.reserved, 0,
         sizeof(debugMonitorSnapshot.reserved));

  for (uint8_t i = 0; i < MAX_KEYS; ++i) {
    copyDebugMonitorLabel(debugMonitorSnapshot.keyNames[i],
                          keysGetLabel((EnumKeys)i));
  }

  for (uint8_t i = 0; i < debugMonitorSnapshot.switchCount; ++i) {
    copyDebugMonitorLabel(debugMonitorSnapshot.switchNames[i],
                          switchGetDefaultName(i));
    debugMonitorSnapshot.switchTypes[i] = switchGetHwType(i);
    debugMonitorSnapshot.switchPositions[i] = switchGetPosition(i);
    debugMonitorSnapshot.functionSwitchPhysical[i] = 0;
    debugMonitorSnapshot.functionSwitchLogical[i] = 0;
#if defined(FUNCTION_SWITCHES)
    if (switchIsCustomSwitch(i)) {
      debugMonitorSnapshot.functionSwitches |= (1u << i);
      debugMonitorSnapshot.functionSwitchPhysical[i] =
          switchGetPosition(i) != SWITCH_HW_UP;
      debugMonitorSnapshot.functionSwitchLogical[i] = g_model.cfsState(i);
    }
#endif
  }

  for (uint8_t i = 0; i < debugMonitorSnapshot.analogCount; ++i) {
    copyDebugMonitorLabel(debugMonitorSnapshot.analogNames[i],
                          adcGetInputName(i));
    debugMonitorSnapshot.analogRaw[i] = getAnalogValue(i);
    debugMonitorSnapshot.analogFiltered[i] = anaIn(i);
  }

  for (uint8_t i = 0; i < MAX_INPUTS; ++i) {
    copyDebugMonitorLabel(debugMonitorSnapshot.inputNames[i],
                          g_model.inputNames[i], LEN_INPUT_NAME);
    debugMonitorSnapshot.inputs[i] = anas[i];
  }

  for (uint8_t i = 0; i < MAX_OUTPUT_CHANNELS; ++i) {
    copyDebugMonitorLabel(debugMonitorSnapshot.channelNames[i],
                          g_model.limitData[i].name, LEN_CHANNEL_NAME);
    debugMonitorSnapshot.mixers[i] = ex_chans[i];
    debugMonitorSnapshot.outputs[i] = channelOutputs[i];
  }

  for (uint8_t i = 0; i < NUM_MODULES; ++i) {
    auto &destination = debugMonitorSnapshot.modules[i];
    memset(&destination, 0, sizeof(destination));
    destination.protocol = moduleState[i].protocol;
    destination.powered = modulePortPowered(i);

    auto source = modulePortGetState(i);
    if (source) {
      captureDebugMonitorPort(destination.tx, source->tx, ETX_Dir_TX);
      captureDebugMonitorPort(destination.rx, source->rx, ETX_Dir_RX);
    }
  }

  __DMB();
  debugMonitorSnapshot.sequence = sequence + 1;
}

#endif

#if defined(SIMU)
#define PRINTF_BUFFER_SIZE     1024
void debugPrintf(const char * format, ...)
{
  va_list arglist;
  char tmp[PRINTF_BUFFER_SIZE];

  va_start(arglist, format);
  vsnprintf(tmp, PRINTF_BUFFER_SIZE, format, arglist);
  va_end(arglist);
  fputs(tmp, stdout);
  fflush(stdout);
  if (traceCallback) {
    traceCallback(tmp);
  }
}
#endif

#if defined(DEBUG_INTERRUPTS)

#if defined(PCBHORUS)
  const char * const interruptNames[INT_LAST] = {
    "Tick ",   // INT_TICK,
    "1ms  ",   // INT_1MS,
    "Ser2 ",   // INT_SER2,
    "TelDm",   // INT_TELEM_DMA,
    "TelUs",   // INT_TELEM_USART,
    "Sdio ",   // INT_SDIO,
    "SdDma",   // INT_SDIO_DMA,
    "D2S7 ",   // INT_DMA2S7,
    "Tim1 ",   // INT_TIM1CC,
    "Tim2 ",   // INT_TIM2,
    "Tim3 ",   // INT_TIM3,
    "BlueT",   // INT_BLUETOOTH,
    "USB  ",  // INT_OTG_FS,
#if defined(DEBUG_USB_INTERRUPTS)
    " spur",  // INT_OTG_FS_SPURIOUS,
    "  out",  // INT_OTG_FS_OUT_EP,
    "   in",  // INT_OTG_FS_IN_EP,
    " miss",  // INT_OTG_FS_MODEMISMATCH,
    " wake",  // INT_OTG_FS_WAKEUP,
    " susp",  // INT_OTG_FS_SUSPEND,
    "  sof",  // INT_OTG_FS_SOF,
    " rxst",  // INT_OTG_FS_RX_STAT,
    "  rst",  // INT_OTG_FS_RESET,
    " enum",  // INT_OTG_FS_ENUM,
    " inci",  // INT_OTG_FS_INCOMPLETE_IN,
    " inco",  // INT_OTG_FS_INCOMPLETE_OUT,
    " sess",  // INT_OTG_FS_SESSION,
    "  otg",  // INT_OTG_FS_OTG,
    " notd",  // INT_OTG_FS_RX_NOT_DEVICE,
#endif // #if defined(DEBUG_USB_INTERRUPTS)
  };
#elif defined(PCBTARANIS)
  const char * const interruptNames[INT_LAST] = {
    "Tick ",   // INT_TICK,
    "5ms  ",   // INT_5MS,
    "Audio",   // INT_AUDIO,
    "BlueT",   // INT_BLUETOOTH,
    "Lcd  ",   // INT_LCD,
    "T1CC ",   // INT_TIM1CC,
    "Tim1 ",   // INT_TIM1,
    "Tim8 ",   // INT_TIM8,
    "Ser2 ",   // INT_SER2,
    "TelDm",   // INT_TELEM_DMA,
    "TelUs",   // INT_TELEM_USART,
    "Train",   // INT_TRAINER,
    "Usb  ",   // INT_OTG_FS,
#if defined(DEBUG_USB_INTERRUPTS)
    " spur",  // INT_OTG_FS_SPURIOUS,
    "  out",  // INT_OTG_FS_OUT_EP,
    "   in",  // INT_OTG_FS_IN_EP,
    " miss",  // INT_OTG_FS_MODEMISMATCH,
    " wake",  // INT_OTG_FS_WAKEUP,
    " susp",  // INT_OTG_FS_SUSPEND,
    "  sof",  // INT_OTG_FS_SOF,
    " rxst",  // INT_OTG_FS_RX_STAT,
    "  rst",  // INT_OTG_FS_RESET,
    " enum",  // INT_OTG_FS_ENUM,
    " inci",  // INT_OTG_FS_INCOMPLETE_IN,
    " inco",  // INT_OTG_FS_INCOMPLETE_OUT,
    " sess",  // INT_OTG_FS_SESSION,
    "  otg",  // INT_OTG_FS_OTG,
    " notd",  // INT_OTG_FS_RX_NOT_DEVICE,
#endif // #if defined(DEBUG_USB_INTERRUPTS)
  };
#endif

struct InterruptCounters interruptCounters;
#endif //#if defined(DEBUG_INTERRUPTS)

#if defined(DEBUG_TIMERS)

void DebugTimer::start()
{
  _start_hiprec = getTmr2MHz();
  _start_loprec = get_tmr10ms();
}

void DebugTimer::stop()
{
  // getTmr2MHz is 16 bit timer, resolution 0.5us, max measurable value 32.7675 milli seconds
  // tmr10ms_t tmr10ms = get_tmr10ms(); 32 bit timer, resolution 10ms, max measurable value: 42949672.95 s = 1.3 years
  // if time difference is bigger than 30ms, then use low resolution timer
  // otherwise use high resolution
  if ((_start_hiprec == 0) && (_start_loprec == 0)) return;

  last = get_tmr10ms() - _start_loprec;  //use low precision timer
  if (last < 3) {
    //use high precision
    last = (uint16_t)(getTmr2MHz() - _start_hiprec) / 2;
  }
  else {
    last *= 10000ul; //adjust unit to 1us
  }
  evalStats();
}

DebugTimer debugTimers[DEBUG_TIMERS_COUNT];

const char * const debugTimerNames[DEBUG_TIMERS_COUNT] = {
   "Pulses int."   // debugTimerIntPulses
  ,"Pulses dur."   // debugTimerIntPulsesDuration
  ,"10ms dur.  "   // debugTimerPer10ms
  ,"10ms period"   // debugTimerPer10msPeriod
  ,"Rotary enc."   // debugTimerRotEnc
  ,"Haptic     "   // debugTimerHaptic
  ,"Mixer calc "   // debugTimerMixer
  ,"Tel. wakeup"   // debugTimerTelemetryWakeup
  ,"perMain dur"   // debugTimerPerMain
  ," perMain s1"   // debugTimerPerMain1
  ," guiMain   "   // debugTimerGuiMain
  ,"  LUA      "   // debugTimerLua
  ,"  LCD wait "   // debugTimerLcdRefreshWait
  ,"  LCD refr."   // debugTimerLcdRefresh
  ,"  Menus    "   // debugTimerMenus
  ,"   Menu hnd"   // debugTimerMenuHandlers
  ,"Menu Vers. "   // debugTimerVersion
  ,"Menu simple"   // debugTimerSimpleMenu
  ,"Menu drawte"   // debugTimerDrawText
  ,"Menu drawt1"   // debugTimerDrawText1
  ,"Mix ADC    "   // debugTimerGetAdc
  ,"Mix getsw  "   // debugTimerGetSwitches
  ,"Mix eval   "   // debugTimerEvalMixes
  ,"Mix 10ms   "   // debugTimerMixes10ms
  ,"ADC read   "   // debugTimerAdcRead
  ,"mix-pulses "   // debugTimerMixerCalcToUsage
  ,"mix-int.   "   // debugTimerMixerIterval
  ,"Audio int. "   // debugTimerAudioIterval
  ,"Audio dur. "   // debugTimerAudioDuration
  ," A. consume"   // debugTimerAudioConsume
  ,"SpaceMouse "   // debugTimerSpaceMouseWakeup
};

#endif
