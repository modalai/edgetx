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

#include "stm32_hal_ll.h"
#include "stm32_gpio.h"
#include "stm32_ws2812.h"

#include "hal/switch_driver.h"
#include "hal/module_port.h"
#include "hal/abnormal_reboot.h"
#include "hal/usb_driver.h"
#include "hal/gpio.h"

#include "board.h"
#include "boards/generic_stm32/module_ports.h"
#include "boards/generic_stm32/analog_inputs.h"
#include "boards/generic_stm32/rgb_leds.h"

#include "debug.h"
#include "rtc.h"

#include "timers_driver.h"
#include "dataconstants.h"

#if defined(FLYSKY_GIMBAL)
  #include "flysky_gimbal_driver.h"
#endif

#if !defined(BOOT)
  #include "edgetx.h"
  #if defined(PXX1)
    #include "pulses/pxx1.h"
  #endif
#endif

#if defined(BLUETOOTH)
  #include "bluetooth_driver.h"
#endif

HardwareOptions hardwareOptions;

#if !defined(BOOT)

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles();
#endif


void boardInit()
{
#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#elif defined(DEBUG_SEGGER_RTT)
  SEGGER_RTT_Init();
  // SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif
  //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  SCB_EnableDCache();

  // Setup pll1
  RCC->PLL1DIVR |= (0xA0000U & RCC_PLL1DIVR_Q1_Msk); // Set prescaler to 6 (~21 MHz after scaling)
  RCC->CR |= RCC_CR_PLL1ON;
  while (RCC->CR & RCC_CR_PLL1ON == 0) { }

  // Setup pll2
  RCC->PLLCKSELR &= ~(0x10000 & RCC_PLLCKSELR_DIVM2);
  RCC->PLLCKSELR |= (0x10000 & RCC_PLLCKSELR_DIVM2);
  //DIVN2 = 200
  //DIVP2 = 8
  RCC->PLL2DIVR |= ((0x1000 & RCC_PLL2DIVR_P2) | (0xC8 & RCC_PLL2DIVR_N2));
  RCC->CR |= RCC_CR_PLL2ON;
  while (RCC->CR & RCC_CR_PLL2ON == 0) { }


#if defined(USB_CHARGE_LED) && !defined(DEBUG)
  usbInit();
  // prime debounce state...
  usbPlugged();

  if (usbPlugged()) {
    delaysInit();
#if defined(AUDIO_MUTE_GPIO)
     // Charging can make a buzzing noise
     gpio_init(AUDIO_MUTE_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
     gpio_set(AUDIO_MUTE_GPIO);
 #endif
    while (usbPlugged()) {
      delay_ms(1000);
    }
    pwrOff();
  }
#endif

  void board_set_bor_level();
  board_set_bor_level();

  // Sets 'hardwareOption.pcbrev' as well
  pwrInit();
  boardInitModulePorts();

#if defined(STATUS_LEDS)
  ledInit();
  ledGreen();
#endif

  keysInit();
  switchInit();

#if defined(ROTARY_ENCODER_NAVIGATION)
  rotaryEncoderInit();
#endif

  delaysInit();
  __enable_irq();

  if (!adcInit(&_adc_driver))
    TRACE("adcInit failed");

  lcdInit(); // delaysInit() must be called before

  timersInit();
  // usbInit();

#if defined(DEBUG) && defined(AUX_SERIAL)
  serialSetMode(SP_AUX1, UART_MODE_DEBUG);                // indicate AUX1 is used
  serialInit(SP_AUX1, UART_MODE_DEBUG);                   // early AUX1 init
#endif

#if defined(DEBUG)
  // Freeze timers & watchdog when core is halted
  // DBGMCU->APB1FZ = 0x00E009FF;
  // DBGMCU->APB2FZ = 0x00070003;
  // TODO: define these for the H7
#endif

#if defined(PWR_BUTTON_PRESS) // TODO: re-enable
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    pwrOn();
  }
#endif

#if defined(USB_CHARGER)
  usbChargerInit();
#endif

#if defined(RTCLOCK)
  rtcInit(); // RTC must be initialized before rambackupRestore() is called
#endif

  backlightInit();

  gpio_set(GPIO_PIN(GPIOH, 2));

#if defined(GUI)
  lcdSetContrast(true);
#endif
}
#endif

void boardOff()
{
#if defined(STATUS_LEDS) && !defined(BOOT)
  ledOff();
#endif

  BACKLIGHT_DISABLE();


#if defined(PWR_BUTTON_PRESS)
  while (pwrPressed()) {
    WDG_RESET();
  }
#endif

// #if defined(MANUFACTURER_RADIOMASTER) && defined(STM32F407xx)
//   lcdInit(); 
// #endif

  lcdOff();
  SysTick->CTRL = 0; // turn off systick
  pwrOff();

  // disable interrupts
  __disable_irq();

  while (1) {
    WDG_RESET();
#if defined(PWR_BUTTON_PRESS)
    // X9E/X7 needs watchdog reset because CPU is still running while
    // the power key is held pressed by the user.
    // The power key should be released by now, but we must make sure
    if (!pwrPressed()) {
      // Put the CPU into sleep to reduce the consumption,
      // it might help with the RTC reset issue
      //PWR->CR |= PWR_CR_CWUF;
      /* Select STANDBY mode */
      //PWR->CR |= PWR_CR_PDDS;
      /* Set SLEEPDEEP bit of Cortex System Control Register */
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      /* Request Wait For Event */
      __WFE();
    }
#endif
  }

  // this function must not return!
}
