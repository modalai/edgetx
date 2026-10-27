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
#include "stm32_timer.h"

#include "board.h"

#if !defined(BACKLIGHT_GPIO) && (!defined(BACKLIGHT_GPIO_1) || !defined(BACKLIGHT_GPIO_2))
  // no backlight
  void backlightInit() {}
  void backlightEnable(uint8_t level) {}
  void backlightFullOn() {}
  void backlightDisable() {}
  uint8_t isBacklightEnabled() {return false;}
#elif defined(PCBX9E)
void backlightInit()
{
  gpio_init_af(BACKLIGHT_GPIO_1, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(BACKLIGHT_GPIO_2, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  stm32_timer_enable_clock(BACKLIGHT_TIMER);
  BACKLIGHT_TIMER->ARR = 100;
  BACKLIGHT_TIMER->PSC = BACKLIGHT_TIMER_FREQ / 50000 - 1; // 20us * 100 = 2ms => 500Hz
  BACKLIGHT_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM
  BACKLIGHT_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  BACKLIGHT_TIMER->CCR2 = 0;
  BACKLIGHT_TIMER->CCR1 = 100;
  BACKLIGHT_TIMER->EGR = 0;
  BACKLIGHT_TIMER->CR1 = TIM_CR1_CEN; // Counter enable
}

void backlightEnable(uint8_t level, uint8_t color)
{
  BACKLIGHT_TIMER->CCR1 = ((100-level)*(20-color))/20;
  BACKLIGHT_TIMER->CCR2 = ((100-level)*color)/20;
}

void backlightFullOn()
{
  backlightEnable(0, 0);
}

void backlightDisable()
{
  BACKLIGHT_TIMER->CCR1 = 0;
  BACKLIGHT_TIMER->CCR2 = 0;
}

uint8_t isBacklightEnabled()
{
  return (BACKLIGHT_TIMER->CCR1 != 0 || BACKLIGHT_TIMER->CCR2 != 0);
}
#elif defined(PCBX9DP)
void backlightInit()
{
  gpio_init_af(BACKLIGHT_GPIO_1, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(BACKLIGHT_GPIO_2, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  stm32_timer_enable_clock(BACKLIGHT_TIMER);
  BACKLIGHT_TIMER->ARR = 100;
  BACKLIGHT_TIMER->PSC = BACKLIGHT_TIMER_FREQ / 50000 - 1; // 20us * 100 = 2ms => 500Hz
  BACKLIGHT_TIMER->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM
  BACKLIGHT_TIMER->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM
  BACKLIGHT_TIMER->CCER = TIM_CCER_CC4E | TIM_CCER_CC2E;
  BACKLIGHT_TIMER->CCR2 = 0;
  BACKLIGHT_TIMER->CCR4 = 0;
  BACKLIGHT_TIMER->EGR = 0;
  BACKLIGHT_TIMER->CR1 = TIM_CR1_CEN; // Counter enable
}

void backlightEnable(uint8_t level, uint8_t color)
{
  BACKLIGHT_TIMER->CCR4 = ((100-level)*(20-color))/20;
  BACKLIGHT_TIMER->CCR2 = ((100-level)*color)/20;
}

void backlightFullOn()
{
  backlightEnable(0, 0);
}

void backlightDisable()
{
  BACKLIGHT_TIMER->CCR4 = 0;
  BACKLIGHT_TIMER->CCR2 = 0;
}

uint8_t isBacklightEnabled()
{
  return (BACKLIGHT_TIMER->CCR4 != 0 || BACKLIGHT_TIMER->CCR2 != 0);
}
#elif defined(PCBX7) || defined(PCBXLITE) || defined(PCBX9LITE) || defined(PCBM207) || defined(PCBM196)

#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
namespace {

uint8_t helmBacklightDuty(uint8_t level)
{
  const uint16_t brightness = level >= 100 ? 0 : 100 - level;
  return (brightness * brightness + 99) / 100;
}

void connectBacklightPwm()
{
  gpio_init_af(BACKLIGHT_GPIO, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
}

void disconnectBacklightPwm()
{
  BACKLIGHT_COUNTER_REGISTER = 0;
  gpio_clear(BACKLIGHT_GPIO);
  gpio_init(BACKLIGHT_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
}

}  // namespace
#endif

void backlightInit()
{
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
#if defined(BOOT)
  constexpr bool lightsDisabled = false;
#elif defined(PWR_BUTTON_STARTUP_GATE)
  // Start dark. The startup gate lights the screen once the power button
  // press outlasts a plug-in transient, and normal backlight handling takes
  // over after an MCU-only reset.
  constexpr bool lightsDisabled = true;
#else
  const bool lightsDisabled = helmLightsDisabled();
#endif
  if (lightsDisabled) {
    gpio_clear(BACKLIGHT_GPIO);
    gpio_init(BACKLIGHT_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  } else {
    gpio_init_af(BACKLIGHT_GPIO, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  }
#else
  gpio_init_af(BACKLIGHT_GPIO, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
#endif
  stm32_timer_enable_clock(BACKLIGHT_TIMER);

#if defined(BACKLIGHT_BDTR) // TODO perhaps it can be always done
  BACKLIGHT_TIMER->BDTR = BACKLIGHT_BDTR; // Enable outputs
#endif
  BACKLIGHT_TIMER->ARR = 100;
  BACKLIGHT_TIMER->PSC = BACKLIGHT_TIMER_FREQ / 50000 - 1; // 20us * 100 = 2ms => 500Hz
  #if defined(BACKLIGHT_CCMR1)
    BACKLIGHT_TIMER->CCMR1 = BACKLIGHT_CCMR1;
  #elif defined(BACKLIGHT_CCMR2)
    BACKLIGHT_TIMER->CCMR2 = BACKLIGHT_CCMR2;
  #endif
  BACKLIGHT_TIMER->CCER = BACKLIGHT_CCER;
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
  BACKLIGHT_COUNTER_REGISTER = lightsDisabled ? 0 : 100;
#else
  BACKLIGHT_COUNTER_REGISTER = 100;
#endif
  BACKLIGHT_TIMER->EGR = 0;
  BACKLIGHT_TIMER->CR1 = TIM_CR1_CEN; // Counter enable
}

void backlightEnable(uint8_t level)
{
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
#if !defined(BOOT)
  if (helmLightsDisabled()) {
    disconnectBacklightPwm();
    return;
  }
#endif
  const uint8_t duty = helmBacklightDuty(level);
  if (duty == 0) {
    disconnectBacklightPwm();
    return;
  }

  BACKLIGHT_COUNTER_REGISTER = duty;
  connectBacklightPwm();
#else
  BACKLIGHT_COUNTER_REGISTER = 100 - level;
#endif
}

void backlightFullOn()
{
  backlightEnable(0);
}

void backlightDisable()
{
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
  disconnectBacklightPwm();
#else
  BACKLIGHT_COUNTER_REGISTER = 0;
#endif
}

uint8_t isBacklightEnabled()
{
  return BACKLIGHT_COUNTER_REGISTER != 0;
}
#else
void backlightInit()
{
  gpio_init_af(BACKLIGHT_GPIO, BACKLIGHT_GPIO_AF, GPIO_PIN_SPEED_LOW);
  stm32_timer_enable_clock(BACKLIGHT_TIMER);  
  
  BACKLIGHT_TIMER->ARR = 100;
  BACKLIGHT_TIMER->PSC = BACKLIGHT_TIMER_FREQ / 50000 - 1; // 20us * 100 = 2ms => 500Hz
  BACKLIGHT_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM
  BACKLIGHT_TIMER->CCER = TIM_CCER_CC1E;
  BACKLIGHT_TIMER->CCR1 = 80;
  BACKLIGHT_TIMER->EGR = 0;
  BACKLIGHT_TIMER->CR1 = 1;
}

void backlightEnable(uint8_t level)
{
  BACKLIGHT_TIMER->CCR1 = 100 - level;
}

void backlightFullOn()
{
  backlightEnable(0);
}

void backlightDisable()
{
  BACKLIGHT_TIMER->CCR1 = 0;
}

uint8_t isBacklightEnabled()
{
  return BACKLIGHT_TIMER->CCR1 != 0;
}
#endif
