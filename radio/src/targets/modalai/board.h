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

#pragma once

#include <inttypes.h>
#include "hal.h"
#include "hal/serial_port.h"
#include "hal/watchdog_driver.h"

#include "definitions.h"
#include "edgetx_constants.h"
#include "board_common.h"

#if defined(ROTARY_ENCODER_NAVIGATION)
// Rotary Encoder driver
void rotaryEncoderInit();
void rotaryEncoderCheck();
#endif

#define FLASHSIZE                       0x80000  // 512k
#define FLASH_PAGESIZE                  256
#define BOOTLOADER_SIZE                 0x10000
#define FIRMWARE_ADDRESS                0x08000000
#define FIRMWARE_LEN(fsize)             (fsize - BOOTLOADER_SIZE)
#define FIRMWARE_MAX_LEN                (FLASHSIZE - BOOTLOADER_SIZE)
#define APP_START_ADDRESS               (uint32_t)(FIRMWARE_ADDRESS + BOOTLOADER_SIZE)

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited


extern uint16_t sessionTimer;

// Board driver
void boardInit();
void boardOff();

// PCBREV driver
enum {
  // X7
  PCBREV_X7_STD = 0,
  PCBREV_X7_40 = 1,
};

// Pulses driver
//#define INTERNAL_MODULE_ON()   gpio_set(INTMODULE_PWR_GPIO)
//#define INTERNAL_MODULE_OFF()  gpio_clear(INTMODULE_PWR_GPIO)
#define INTERNAL_MODULE_ON()
#define INTERNAL_MODULE_OFF()

#if (defined(INTERNAL_MODULE_PXX1) || defined(INTERNAL_MODULE_PXX2)) && (!defined(PCBX9LITE) || defined(PCBX9LITES))
  #define HARDWARE_INTERNAL_RAS
#endif

//#define EXTERNAL_MODULE_ON()            EXTERNAL_MODULE_PWR_ON()
//#define EXTERNAL_MODULE_OFF()           EXTERNAL_MODULE_PWR_OFF()
#define EXTERNAL_MODULE_ON()
#define EXTERNAL_MODULE_OFF()

// Trainer driver
#define SLAVE_MODE()                    (g_model.trainerData.mode == TRAINER_MODE_SLAVE)

// Not supporting a trainer
#define TRAINER_CONNECTED()           false

#define NUM_FUNCTIONS_SWITCHES 0

PACK(typedef struct {
  uint8_t pcbrev:2;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;

// NI-MH 7.2V
// #define BATTERY_WARN                  65 // 6.5V
// #define BATTERY_MIN                   60 // 6.0V
// #define BATTERY_MAX                   80 // 8.0V
#define BATTERY_WARN                  65 // 6.5V
#define BATTERY_MIN                   60 // 6.0V
#define BATTERY_MAX                   80 // 8.0V

#define BATT_SCALE                    123

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver

void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
bool pwrOffPressed();
void pwrResetHandler();
#define pwrForcePressed()   false

// Backlight driver
#if defined(OLED_SCREEN)
#define BACKLIGHT_DISABLE()             lcdSetRefVolt(0)
#define BACKLIGHT_FORCED_ON             255
#else
#define BACKLIGHT_DISABLE()             backlightDisable()
#define BACKLIGHT_FORCED_ON             101
#endif


void backlightInit();
void backlightDisable();
void backlightFullOn();
uint8_t isBacklightEnabled();

#if defined(PCBX9E) || defined(PCBX9DP)
  void backlightEnable(uint8_t level, uint8_t color);
  #define BACKLIGHT_ENABLE() \
    backlightEnable(currentBacklightBright, g_eeGeneral.backlightColor)
#elif defined(OLED_SCREEN)
  #define BACKLIGHT_ENABLE() lcdSetRefVolt(currentBacklightBright)
#else
  void backlightEnable(uint8_t level);
  #define BACKLIGHT_ENABLE() backlightEnable(currentBacklightBright)
#endif

#if defined(__cplusplus) && !defined(SIMU)
}
#endif

// Debug driver
void debugPutc(const char c);

// Audio driver
void audioInit() ;
void audioEnd() ;
void dacStart();
void dacStop();
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
#if !defined(SOFTWARE_VOLUME)
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume();
#endif
#if defined(AUDIO_SPEAKER_ENABLE_GPIO)
void initSpeakerEnable();
void enableSpeaker();
void disableSpeaker();
#else
static inline void initSpeakerEnable() { }
static inline void enableSpeaker() { }
static inline void disableSpeaker() { }
#endif
#if defined(HEADPHONE_TRAINER_SWITCH_GPIO)
void initHeadphoneTrainerSwitch();
void enableHeadphone();
void enableTrainer();
#else
static inline void initHeadphoneTrainerSwitch() { }
static inline void enableHeadphone() { }
static inline void enableTrainer() { }
#endif
#if defined(JACK_DETECT_GPIO)
void initJackDetect();
bool isJackPlugged();
#endif
void audioConsumeCurrentBuffer();
#define audioDisableIrq()               __disable_irq()
#define audioEnableIrq()                __enable_irq()

// Haptic driver
// void hapticInit();
// void hapticOff();
// #if defined(HAPTIC_PWM)
//   void hapticOn(uint32_t pwmPercent);
// #else
//   void hapticOn();
// #endif

#define DEBUG_BAUDRATE                  115200
#define LUA_DEFAULT_BAUDRATE            115200

const etx_serial_port_t* auxSerialGetPort(int port_nr);

// USB Charger
#if defined(USB_CHARGER)
void usbChargerInit();
bool usbChargerLed();
#endif

// LED driver
void ledInit();
void ledOff();
void ledRed();
void ledGreen();
void ledBlue();

// LCD driver
#define LCD_W                           128
#define LCD_H                           64
#define LCD_DEPTH                       1
#define IS_LCD_RESET_NEEDED()           true
#define LCD_CONTRAST_MIN                10
#define LCD_CONTRAST_MAX                30
#define LCD_CONTRAST_DEFAULT          20

#define IS_LCD_RESET_NEEDED()           true

void lcdInit();
void lcdInitFinish();
void lcdOff();

// TODO lcdRefreshWait() stub in simpgmspace
#if defined(LCD_DMA) && !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
void lcdRefresh(bool wait=true); // TODO uint8_t wait to simplify this
void lcdSetRefVolt(unsigned char val);
#if LCD_W == 128
void lcdSetInvert(bool invert);
#endif
#ifdef __cplusplus
void lcdSetContrast(bool useDefault = false);
#endif
void lcdFlushed();


#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE             128
#else
#define TELEMETRY_FIFO_SIZE             64
#endif

#define INTMODULE_FIFO_SIZE            128


#define BATTERY_DIVIDER 23711 // = 2047*128*BATT_SCALE/(100*(VREF*(160+499)/160))

#define VOLTAGE_DROP 45

#define NUM_TRIMS                               4

#define NUM_TRIMS_KEYS                          (NUM_TRIMS * 2)

