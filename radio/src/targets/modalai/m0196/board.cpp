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
#include "system_clock.h"
#include "timers_driver.h"

#if !defined(BOOT)
#include "edgetx.h"
#endif

#if defined(HELM_DIAGNOSTICS)
#include "diagnostics.h"
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
  gpio_init(HELM_BOOTLOADER_KEY_GPIO, GPIO_IN_PU, GPIO_PIN_SPEED_LOW);
  return !gpio_read(HELM_BOOTLOADER_KEY_GPIO);
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
  // Keep unverified power and RF control GPIOs in their reset state.
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

  // Power latch polarity is intentionally unverified. Never drive PI6 here.
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
