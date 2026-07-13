/*
 * ModalAI M0196 / Helm board contract.
 */

#pragma once

#include <inttypes.h>

#include "hal.h"
#include "hal/serial_port.h"
#include "hal/watchdog_driver.h"
#include "definitions.h"
#include "edgetx_constants.h"
#include "board_common.h"

#define FLASHSIZE 0x200000
#define FLASH_PAGESIZE 256
#define BOOTLOADER_SIZE 0x20000
#define FIRMWARE_ADDRESS 0x08000000
#define FIRMWARE_LEN(fsize) (fsize - BOOTLOADER_SIZE)
#define FIRMWARE_MAX_LEN (FLASHSIZE - BOOTLOADER_SIZE)
#define APP_START_ADDRESS (uint32_t)(FIRMWARE_ADDRESS + BOOTLOADER_SIZE)

#define LUA_MEM_MAX 0

extern uint16_t sessionTimer;

void boardInit();
void boardOff();

enum {
  PCBREV_HELM_BASIC = 0,
};

PACK(typedef struct {
  uint8_t pcbrev : 2;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;

// Power and RF power outputs remain no-ops until their polarity is verified.
#define INTERNAL_MODULE_ON()
#define INTERNAL_MODULE_OFF()
#define EXTERNAL_MODULE_ON()
#define EXTERNAL_MODULE_OFF()

#define SLAVE_MODE() (g_model.trainerData.mode == TRAINER_MODE_SLAVE)
#define TRAINER_CONNECTED() false
#define NUM_FUNCTIONS_SWITCHES 0

#define BATTERY_WARN 65
#define BATTERY_MIN 60
#define BATTERY_MAX 80
#define BATT_SCALE 123
#define BATTERY_DIVIDER 23711
#define VOLTAGE_DROP 0

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

#define SOFT_PWR_CTRL
void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
bool pwrOffPressed();
void pwrResetHandler();
#define pwrForcePressed() false

void backlightInit();
void backlightDisable();
void backlightFullOn();
void backlightEnable(uint8_t level);
uint8_t isBacklightEnabled();
#define BACKLIGHT_DISABLE() backlightDisable()
#define BACKLIGHT_FORCED_ON 101
#define BACKLIGHT_ENABLE() backlightEnable(currentBacklightBright)

#if defined(HAPTIC)
void hapticInit();
void hapticOff();
void hapticOn(uint32_t pwmPercent);
#endif

#if defined(__cplusplus) && !defined(SIMU)
}
#endif

void debugPutc(const char c);

void audioInit();
void audioEnd();
void dacStart();
void dacStop();
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX 23
#define VOLUME_LEVEL_DEF 12
static inline void initSpeakerEnable() {}
static inline void enableSpeaker() {}
static inline void disableSpeaker() {}
static inline void initHeadphoneTrainerSwitch() {}
static inline void enableHeadphone() {}
static inline void enableTrainer() {}
void audioConsumeCurrentBuffer();
#define audioDisableIrq() __disable_irq()
#define audioEnableIrq() __enable_irq()

#define DEBUG_BAUDRATE 115200
#define LUA_DEFAULT_BAUDRATE 115200
const etx_serial_port_t *auxSerialGetPort(int port_nr);

void ledInit();
void ledOff();
void ledRed();
void ledGreen();
void ledBlue();

#define LCD_W 128
#define LCD_H 64
#define LCD_DEPTH 1
#define IS_LCD_RESET_NEEDED() true
#define LCD_CONTRAST_MIN 10
#define LCD_CONTRAST_MAX 30
#define LCD_CONTRAST_DEFAULT 20

void lcdInit();
void lcdInitFinish();
void lcdOff();
void lcdRefresh(bool wait = false);
void lcdRefreshWait();
void lcdSetRefVolt(unsigned char val);
void lcdSetInvert(bool invert);
uint32_t lcdGetRefreshCount();
uint32_t lcdGetDmaErrorCount();
#ifdef __cplusplus
void lcdSetContrast(bool useDefault = false);
#endif
void lcdFlushed();

#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE 128
#else
#define TELEMETRY_FIFO_SIZE 64
#endif
#define INTMODULE_FIFO_SIZE 128

#define NUM_TRIMS 4
#define NUM_TRIMS_KEYS (NUM_TRIMS * 2)
