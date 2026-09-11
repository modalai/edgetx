#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_timer.h"

#include "board.h"

#if defined(HELM_HAS_RGB_LED)

static void setRgb(uint8_t red, uint8_t green, uint8_t blue)
{
  RGB_LED_TIMER->CCR1 = blue;
  RGB_LED_TIMER->CCR2 = red;
  RGB_LED_TIMER->CCR3 = green;
}

void ledInit()
{
  gpio_init_af(RGB_LED_BLUE_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(RGB_LED_RED_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(RGB_LED_GREEN_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);

  stm32_timer_enable_clock(RGB_LED_TIMER);
  RGB_LED_TIMER->ARR = 100;
  RGB_LED_TIMER->PSC = RGB_LED_TIMER_FREQ / 50000 - 1;
  RGB_LED_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                         TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  RGB_LED_TIMER->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  RGB_LED_TIMER->CCER = RGB_LED_CCER;
  setRgb(0, 0, 0);
  RGB_LED_TIMER->EGR = TIM_EGR_UG;
  RGB_LED_TIMER->CR1 = TIM_CR1_CEN;
}

void ledOff()
{
  setRgb(0, 0, 0);
}

void ledRed()
{
  setRgb(100, 0, 0);
}

void ledGreen()
{
  setRgb(0, 100, 0);
}

void ledBlue()
{
  setRgb(0, 0, 100);
}

#else

void ledInit() {}
void ledOff() {}
void ledRed() {}
void ledGreen() {}
void ledBlue() {}

#endif
