#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_ll_rcc.h"

#include "board.h"

#include "bootloader/boot.h"
#include "boards/generic_stm32/analog_inputs.h"
#include "boards/generic_stm32/module_ports.h"
#include "debug.h"
#include "hal/abnormal_reboot.h"
#include "hal/gpio.h"
#include "hal/module_port.h"
#include "hal/switch_driver.h"
#include "hal/usb_driver.h"
#include "rtc.h"
#include "stm32_gpio.h"
#include "stm32_gpio_driver.h"
#include "stm32h7xx_ll_gpio.h"
#include "system_clock.h"
#include "timers_driver.h"

#if !defined(BOOT)
#include "edgetx.h"
#endif

#if defined(HELM_DIAGNOSTICS)
#include "diagnostics.h"
#endif

#if defined(HELM_HAS_OB_POWER_MONITOR)
  #include "voxlpm_i2c_driver.h"
#endif
#if defined(HELM_HAS_BATTERY_BOARD)
  #warning "M0196 battery board is populated, but its driver is not implemented"
#endif
#if defined(HELM_HAS_I2C_GIMBALS)
  #warning "M0196 I2C gimbals are selected, but their driver is not implemented"
#endif
#if defined(HELM_HAS_OB_EEPROM)
  #warning "M0196 onboard EEPROM is populated, but its driver is not implemented"
#endif
#if defined(HELM_HAS_EXPANSION_PORT)
  #warning "M0196 expansion port is populated, but its module driver is not implemented"
#endif
#if defined(HELM_HAS_INTERNAL_MODULE)
  #warning "M0196 internal radio UART4 is active, but its auxiliary interfaces are not implemented"
#endif
#if defined(HELM_HAS_INTERNAL_FAN)
  #warning "M0196 internal fan is populated, but its driver is not implemented"
#endif
#if defined(HELM_HAS_TRAINER_PORT)
  #warning "M0196 trainer port is populated, but its driver is not implemented"
#endif
#if defined(HELM_HAS_INTERNAL_VRX)
  #warning "M0196 internal VRX is populated, but its driver is not implemented"
#endif
#if defined(HELM_HAS_HALL_SENSOR)
  #warning "M0196 Hall sensor is populated, but its I2C driver is not implemented"
#endif
#if defined(HELM_HAS_INTERNAL_IMU)
  #warning "M0196 internal IMU is populated, but its driver is not implemented"
#endif

HardwareOptions hardwareOptions;
static bool hseClockFailureRecovered;

extern uint32_t _reboot_cmd;
extern "C" void default_isr_handler();

extern "C" void NMI_Handler()
{
  if (LL_RCC_IsActiveFlag_HSECSS()) {
    _reboot_cmd = HELM_REBOOT_CMD_HSE_FAILURE;
    LL_RCC_ClearFlag_HSECSS();
    NVIC_SystemReset();
  }

  default_isr_handler();
}

bool boardBLStartCondition()
{
#if defined(HELM_BOOTLOADER_INPUT)
  auto gpio = HELM_INPUT_GPIO(HELM_BOOTLOADER_INPUT);
  auto pin = HELM_INPUT_PIN(HELM_BOOTLOADER_INPUT);

  stm32_gpio_enable_clock(gpio);
  LL_GPIO_SetPinMode(gpio, pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(gpio, pin, LL_GPIO_PULL_UP);
  return !LL_GPIO_IsInputPinSet(gpio, pin);
#else
  return false;
#endif
}

void boardBLPreJump()
{
#if defined(HELM_DIAGNOSTICS)
  if (hseClockFailureRecovered) {
    TRACE("HELM: recovered from HSE clock failure");
  }
  TRACE("HELM: bootloader jumping to application");
#endif
}

void boardBLInit()
{
#if defined(HELM_DIAGNOSTICS)
  helmDiagnosticsBootloader();
#endif
}

void boardBLEarlyInit()
{
#if defined(HELM_HAS_SOFT_POWER)
  // Assert the latch before the operator releases the momentary button.
  pwrOn();
#endif

  if (abnormalRebootGetCmd() == HELM_REBOOT_CMD_HSE_FAILURE) {
    hseClockFailureRecovered = true;
    abnormalRebootResetCmd();
  }
}

void boardInit()
{
  __enable_irq();

  pwrInit();
  boardInitModulePorts();

  ledInit();
  ledGreen();

  keysInit();
  switchInit();
  delaysInit();

  if (!adcInit(&_adc_driver)) {
    TRACE("HELM: adcInit failed");
  }

  timersInit();
#if defined(HELM_HAS_OB_POWER_MONITOR)
  voxl_pm_init();
#endif
  lcdInit();
  usbInit();

#if defined(RTCLOCK)
  // RTCSEL cannot change without a backup-domain reset. Preserve an existing
  // LSE RTC, but recover boards that were previously configured to use LSI.
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_RTC_SOURCE() != RCC_RTCCLKSOURCE_LSE) {
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
  }
  rtcInit();
#endif

  backlightInit();

#if defined(HAPTIC)
  hapticInit();
#endif

#if defined(GUI)
  lcdSetContrast(true);
#endif

#if defined(HELM_DIAGNOSTICS)
  helmDiagnosticsInit();
#endif
}

void boardOff()
{
  ledOff();
  BACKLIGHT_DISABLE();
#if defined(HAPTIC)
  hapticOff();
#endif
  lcdOff();

#if defined(HELM_HAS_SOFT_POWER)
  while (pwrPressed()) {
    WDG_RESET();
  }
  pwrOff();
#endif

  SysTick->CTRL = 0;
  __disable_irq();
  while (1) {
    WDG_RESET();
    __WFE();
  }
}

#if !defined(BOOT)
void per5ms()
{
#if defined(HAPTIC)
  DEBUG_TIMER_START(debugTimerHaptic);
  HAPTIC_HEARTBEAT();
  DEBUG_TIMER_STOP(debugTimerHaptic);
#endif
#if defined(HELM_DIAGNOSTICS)
  helmDiagnosticsPer5ms();
#endif
}
#endif
