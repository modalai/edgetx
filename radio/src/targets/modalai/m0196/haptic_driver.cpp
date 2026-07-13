/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_timer.h"

#include "board.h"

static void hapticSetDuty(uint32_t pwmPercent)
{
  HAPTIC_TIMER->CCR1 = pwmPercent;
  HAPTIC_TIMER->CCR2 = pwmPercent;
  HAPTIC_TIMER->CCR3 = pwmPercent;
  HAPTIC_TIMER->CCR4 = pwmPercent;
}

void hapticOff()
{
  hapticSetDuty(0);
}

void hapticOn(uint32_t pwmPercent)
{
  if (pwmPercent > 100) {
    pwmPercent = 100;
  }

  hapticSetDuty(pwmPercent);
}

void hapticInit()
{
  stm32_timer_enable_clock(HAPTIC_TIMER);

  HAPTIC_TIMER->CR1 = 0;
  HAPTIC_TIMER->PSC = HAPTIC_TIMER_FREQ / 10000 - 1;
  HAPTIC_TIMER->ARR = 100;
  HAPTIC_TIMER->CCMR1 = HAPTIC_CCMR1;
  HAPTIC_TIMER->CCMR2 = HAPTIC_CCMR2;
  HAPTIC_TIMER->CCER = HAPTIC_CCER;
  hapticSetDuty(0);
  HAPTIC_TIMER->EGR = TIM_EGR_UG;
  HAPTIC_TIMER->SR = 0;
  HAPTIC_TIMER->CR1 = TIM_CR1_CEN;

  // Connect the pins only after all timer outputs have been forced to 0%.
  gpio_init_af(HAPTIC_LEFT_BOTTOM_GPIO, HAPTIC_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(HAPTIC_LEFT_TOP_GPIO, HAPTIC_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(HAPTIC_RIGHT_BOTTOM_GPIO, HAPTIC_GPIO_AF,
               GPIO_PIN_SPEED_LOW);
  gpio_init_af(HAPTIC_RIGHT_TOP_GPIO, HAPTIC_GPIO_AF, GPIO_PIN_SPEED_LOW);
}
