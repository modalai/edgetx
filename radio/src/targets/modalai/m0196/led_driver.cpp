#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_timer.h"

#include "board.h"

#if defined(HELM_HAS_INTERNAL_FAN) && !defined(HELM_HAS_RGB_LED)
#error "The M0196 fan driver requires the shared TIM5 RGB timer"
#endif

#if defined(FUNCTION_SWITCHES)

void fsLedOff(uint8_t index)
{
  (void)index;
}

void fsLedOn(uint8_t index)
{
  (void)index;
}

bool fsLedState(uint8_t index)
{
  (void)index;
  return false;
}

#endif

#if defined(HELM_HAS_RGB_LED)

namespace {

uint8_t rgbRed;
uint8_t rgbGreen;
uint8_t rgbBlue;
uint8_t rgbBrightness = 100;

uint8_t scaleRgb(uint8_t value)
{
  return (static_cast<uint16_t>(value) * rgbBrightness + 50) / 100;
}

void applyRgb()
{
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS) && !defined(BOOT)
  if (helmLightsDisabled()) {
    RGB_LED_TIMER->CCR1 = 0;
    RGB_LED_TIMER->CCR2 = 0;
    RGB_LED_TIMER->CCR3 = 0;
    return;
  }
#endif
  RGB_LED_TIMER->CCR1 = scaleRgb(rgbBlue);
  RGB_LED_TIMER->CCR2 = scaleRgb(rgbRed);
  RGB_LED_TIMER->CCR3 = scaleRgb(rgbGreen);
}

}  // namespace

static void setRgb(uint8_t red, uint8_t green, uint8_t blue)
{
  rgbRed = red;
  rgbGreen = green;
  rgbBlue = blue;
  applyRgb();
}

#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
void ledSetBrightness(uint8_t percent)
{
  if (percent > 100) percent = 100;
  if (rgbBrightness == percent) return;

  rgbBrightness = percent;
  applyRgb();
}
#endif

#if defined(HELM_HAS_INTERNAL_FAN)

namespace {

constexpr uint8_t FAN_START_DUTY = 100;
constexpr uint8_t FAN_START_TICKS_10MS = 25;
// Lowest duty that keeps the fan turning. Fan Strength 1-100 % maps onto
// FAN_MIN_DUTY-100 %, so every slider step changes the speed. 0 is linear.
constexpr uint8_t FAN_MIN_DUTY = 0;

uint8_t fanDuty;
uint8_t fanStartTicks;

void setFanPwm(uint8_t percent)
{
  RGB_LED_TIMER->CCR4 =
      percent >= 100 ? RGB_LED_TIMER->ARR + 1 : percent;
}

}  // namespace

uint8_t fanDutyFromPercent(uint8_t percent)
{
  if (percent == 0) return 0;
  return FAN_MIN_DUTY + ((100 - FAN_MIN_DUTY) * percent + 50) / 100;
}

void fanSetPower(uint8_t percent)
{
  if (percent > 100) percent = 100;
  percent = fanDutyFromPercent(percent);

  const bool start = fanDuty == 0 && percent > 0;
  fanDuty = percent;
  fanStartTicks = start ? FAN_START_TICKS_10MS : 0;
  setFanPwm(start ? FAN_START_DUTY : fanDuty);
}

void fanTick10ms()
{
  if (fanStartTicks == 0) return;

  if (--fanStartTicks == 0) {
    setFanPwm(fanDuty);
  }
}

#endif

void ledInit()
{
  gpio_init_af(RGB_LED_BLUE_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(RGB_LED_RED_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);
  gpio_init_af(RGB_LED_GREEN_GPIO, RGB_LED_GPIO_AF, GPIO_PIN_SPEED_LOW);

  stm32_timer_enable_clock(RGB_LED_TIMER);
  RGB_LED_TIMER->ARR = 100;
  // 2.5 MHz ticks with ARR = 100 give ~24.75 kHz PWM at 1 % steps. The fan is a
  // 2-wire BLDC switched on its low side; at the old ~495 Hz every off gap
  // browned out its internal driver and it stalled even at 85-95 % duty.
  // The RGB channels on this timer are unaffected by the higher frequency.
  RGB_LED_TIMER->PSC = RGB_LED_TIMER_FREQ / 2500000 - 1;
  RGB_LED_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                         TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  RGB_LED_TIMER->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  RGB_LED_TIMER->CCER = RGB_LED_CCER;
#if defined(HELM_HAS_INTERNAL_FAN)
  RGB_LED_TIMER->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  RGB_LED_TIMER->CCER |= TIM_CCER_CC4E;
  setFanPwm(0);
#endif
  setRgb(0, 0, 0);
  RGB_LED_TIMER->EGR = TIM_EGR_UG;
  RGB_LED_TIMER->CR1 = TIM_CR1_CEN;

#if defined(HELM_HAS_INTERNAL_FAN)
  gpio_clear(HELM_FAN_OUTPUT);
  gpio_init(HELM_FAN_OUTPUT, GPIO_OD_PU, GPIO_PIN_SPEED_LOW);
  gpio_set_af(HELM_FAN_OUTPUT, HELM_FAN_OUTPUT_AF);
#endif
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
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS)
void ledSetBrightness(uint8_t percent) { (void)percent; }
#endif

#endif
