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
#include "stm32_hal_ll.h"
#include "stm32h7xx_ll_gpio.h"
#include "system_clock.h"
#include "timers_driver.h"

#if !defined(BOOT)
#include "edgetx.h"
#endif

#if defined(HELM_DIAGNOSTICS)
#include "diagnostics.h"
#endif

#if defined(HELM_HOST_KEY_INJECTION) && defined(DEBUG_SEGGER_RTT)
#include "rtt_key_injection.h"
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
#if defined(HELM_HAS_OB_EEPROM) && !defined(BOOT)
  #include "eeprom_93aa66.h"
#endif
#if defined(HELM_FACTORY_READ_ONLY_STORAGE) && !defined(BOOT)
  #include "storage/helm_device_settings.h"
#endif
#if defined(HELM_HAS_EXPANSION_PORT)
  #warning "M0196 expansion port is populated, but its module driver is not implemented"
#endif
#if defined(HELM_HAS_INTERNAL_MODULE)
  #warning "M0196 internal radio UART4 is active, but its auxiliary interfaces are not implemented"
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

#if defined(HELM_HAS_SOFT_POWER)
namespace {

constexpr uint32_t POWER_GATE_REBOOT_CMD = 0x50574754U;  // "PWGT"
constexpr uint32_t POWER_ON_HOLD_MS = 1000;
// Applying power charges C37 on the button node, which reads as a short
// press. Keep the backlight off until a press outlasts that transient.
constexpr uint32_t POWER_ON_BACKLIGHT_DELAY_MS = 150;
constexpr uint32_t POWER_ON_SAMPLE_MS = 10;
constexpr uint32_t POWER_ON_DISPLAY_REFRESH_MS = 50;

}  // namespace
#endif

#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS) && !defined(BOOT)
namespace {

constexpr uint8_t LIGHT_BRIGHTNESS_STEP = 10;
constexpr uint8_t LIGHT_BUTTON_DEBOUNCE_TICKS = 3;
constexpr uint8_t LIGHT_BUTTON_REPEAT_DELAY_TICKS = 50;
constexpr uint8_t LIGHT_BUTTON_REPEAT_TICKS = 10;

struct LightButtonState {
  bool stablePressed;
  uint8_t debounceTicks;
  uint8_t repeatTicks;
};

LightButtonState lightUpButton{};
LightButtonState lightDownButton{};

bool lightButtonEvent(LightButtonState& state, bool pressed)
{
  if (pressed != state.stablePressed) {
    if (++state.debounceTicks < LIGHT_BUTTON_DEBOUNCE_TICKS) return false;

    state.stablePressed = pressed;
    state.debounceTicks = 0;
    state.repeatTicks = 0;
    return pressed;
  }

  state.debounceTicks = 0;
  if (!pressed) return false;

  if (state.repeatTicks < LIGHT_BUTTON_REPEAT_DELAY_TICKS) {
    ++state.repeatTicks;
    return false;
  }

  state.repeatTicks =
      LIGHT_BUTTON_REPEAT_DELAY_TICKS - LIGHT_BUTTON_REPEAT_TICKS + 1;
  return true;
}

void initLightBrightnessButtons()
{
  LL_GPIO_InitTypeDef init;
  LL_GPIO_StructInit(&init);
  init.Mode = LL_GPIO_MODE_INPUT;
  init.Pull = LL_GPIO_PULL_UP;

  stm32_gpio_enable_clock(HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_UP));
  init.Pin = HELM_INPUT_PIN(HELM_LIGHT_BRIGHTNESS_UP);
  LL_GPIO_Init(HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_UP), &init);

  stm32_gpio_enable_clock(HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_DOWN));
  init.Pin = HELM_INPUT_PIN(HELM_LIGHT_BRIGHTNESS_DOWN);
  LL_GPIO_Init(HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_DOWN), &init);
}

void changeLightBrightness(int8_t change)
{
  int16_t brightness = 100 - g_eeGeneral.backlightBright + change;
  brightness = limit<int16_t>(0, brightness, 100);
  const uint8_t storedBrightness = 100 - brightness;
  if (g_eeGeneral.backlightBright == storedBrightness) return;

  g_eeGeneral.backlightBright = storedBrightness;
  requiredBacklightBright = storedBrightness;
  resetBacklightTimeout();
  storageDirty(EE_GENERAL);
}

}  // namespace

bool helmLightsDisabled()
{
  return g_eeGeneral.backlightBright >= 100;
}

void helmLightBrightnessTick10ms()
{
  const bool upPressed = !LL_GPIO_IsInputPinSet(
      HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_UP),
      HELM_INPUT_PIN(HELM_LIGHT_BRIGHTNESS_UP));
  const bool downPressed = !LL_GPIO_IsInputPinSet(
      HELM_INPUT_GPIO(HELM_LIGHT_BRIGHTNESS_DOWN),
      HELM_INPUT_PIN(HELM_LIGHT_BRIGHTNESS_DOWN));
  const bool upEvent = lightButtonEvent(lightUpButton, upPressed);
  const bool downEvent = lightButtonEvent(lightDownButton, downPressed);

  if (upEvent != downEvent) {
    changeLightBrightness(upEvent ? -LIGHT_BRIGHTNESS_STEP
                                  : LIGHT_BRIGHTNESS_STEP);
  }
}
#endif

#if defined(FACTORY_TEST_EXTRA_INPUTS)
namespace {

struct FactoryTestInput {
  const char* name;
  GPIO_TypeDef* gpio;
  uint32_t pin;
};

const FactoryTestInput factoryTestInputs[] = {
  {"UL UP", HELM_INPUT_GPIO(UPR_L_B), HELM_INPUT_PIN(UPR_L_B)},
  {"LL CLICK", HELM_INPUT_GPIO(LOW_L_E), HELM_INPUT_PIN(LOW_L_E)},
  {"LR UP", HELM_INPUT_GPIO(LOW_R_C), HELM_INPUT_PIN(LOW_R_C)},
  {"LR DOWN", HELM_INPUT_GPIO(LOW_R_B), HELM_INPUT_PIN(LOW_R_B)},
  {"LR CLICK", HELM_INPUT_GPIO(LOW_R_E), HELM_INPUT_PIN(LOW_R_E)},
};

constexpr uint8_t FACTORY_TEST_INPUT_COUNT =
    sizeof(factoryTestInputs) / sizeof(factoryTestInputs[0]);

}  // namespace

void boardFactoryTestInitExtraInputs()
{
  LL_GPIO_InitTypeDef init;
  LL_GPIO_StructInit(&init);
  init.Mode = LL_GPIO_MODE_INPUT;
  init.Pull = LL_GPIO_PULL_UP;

  for (const auto& input : factoryTestInputs) {
    stm32_gpio_enable_clock(input.gpio);
    init.Pin = input.pin;
    LL_GPIO_Init(input.gpio, &init);
  }
}

uint8_t boardFactoryTestGetExtraInputCount()
{
#if defined(FACTORY_TEST_POWER_BUTTON)
  return FACTORY_TEST_INPUT_COUNT + 1;
#else
  return FACTORY_TEST_INPUT_COUNT;
#endif
}

const char* boardFactoryTestGetExtraInputName(uint8_t index)
{
  if (index < FACTORY_TEST_INPUT_COUNT) return factoryTestInputs[index].name;
#if defined(FACTORY_TEST_POWER_BUTTON)
  if (index == FACTORY_TEST_INPUT_COUNT) return "POWER";
#endif
  return "";
}

bool boardFactoryTestIsExtraInputActive(uint8_t index)
{
  if (index < FACTORY_TEST_INPUT_COUNT) {
    const auto& input = factoryTestInputs[index];
    return !LL_GPIO_IsInputPinSet(input.gpio, input.pin);
  }
#if defined(FACTORY_TEST_POWER_BUTTON)
  if (index == FACTORY_TEST_INPUT_COUNT) return pwrPressed();
#endif
  return false;
}
#endif

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
  // Hold the LCD in reset before any bootloader delay. Preload the output
  // latch before output mode. This order prevents a high pulse during setup.
  stm32_gpio_enable_clock(gpio_get_port(LCD_RST_GPIO));
  gpio_clear(LCD_RST_GPIO);
  gpio_init(LCD_RST_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);

#if defined(HELM_HAS_INTERNAL_FAN)
  gpio_clear(HELM_FAN_OUTPUT);
  gpio_init(HELM_FAN_OUTPUT, GPIO_OD_PU, GPIO_PIN_SPEED_LOW);
#endif

#if defined(HELM_HAS_SOFT_POWER)
  const bool powerGatePending =
      abnormalRebootGetCmd() == POWER_GATE_REBOOT_CMD;
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() && !powerGatePending) {
    // Keep the existing latch state across an MCU-only reset.
    pwrOn();
  } else {
    // Keep the latch inactive until the application accepts the hold.
    gpio_clear(PWR_ON_GPIO);
    gpio_init(PWR_ON_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  }
#endif

  if (abnormalRebootGetCmd() == HELM_REBOOT_CMD_HSE_FAILURE) {
    hseClockFailureRecovered = true;
    abnormalRebootResetCmd();
  }
}

void boardBLPowerOnGate()
{
#if defined(HELM_HAS_SOFT_POWER)
  const bool powerGatePending =
      abnormalRebootGetCmd() == POWER_GATE_REBOOT_CMD;
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() && !powerGatePending) {
    pwrOn();
    return;
  }

  while (!pwrPressed()) {
    pwrOff();
    delay_ms(POWER_ON_SAMPLE_MS);
  }

  _reboot_cmd = POWER_GATE_REBOOT_CMD;
#endif
}

void boardInit()
{
  __enable_irq();

  pwrInit();
  boardInitModulePorts();

  keysInit();
  switchInit();
#if defined(HELM_HAS_SHARED_LIGHT_BRIGHTNESS) && !defined(BOOT)
  initLightBrightnessButtons();
#endif
  delaysInit();

  if (!adcInit(&_adc_driver)) {
    TRACE("HELM: adcInit failed");
  }

  timersInit();
#if defined(HELM_HAS_OB_EEPROM) && !defined(BOOT)
  helmEepromInit();
#if defined(HELM_FACTORY_READ_ONLY_STORAGE)
  helmDeviceSettingsPrepareLightLock();
#endif
#endif

  ledInit();
  ledGreen();
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

#if defined(PWR_BUTTON_STARTUP_GATE) && !defined(BOOT)
void boardPowerOnStartupGate()
{
  const bool powerGatePending =
      abnormalRebootGetCmd() == POWER_GATE_REBOOT_CMD;
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() && !powerGatePending) {
    pwrOn();
    return;
  }

  const uint32_t started = timersGetMsTick();
  uint32_t nextRefresh = 0;
  bool accepted = false;
  bool backlightOn = false;

  while (pwrPressed()) {
    const uint32_t elapsed = timersGetMsTick() - started;
    if (elapsed >= POWER_ON_HOLD_MS) {
      accepted = true;
      break;
    }

    if (!backlightOn && elapsed >= POWER_ON_BACKLIGHT_DELAY_MS) {
      backlightFullOn();
      backlightOn = true;
    }

    if (elapsed >= nextRefresh) {
      drawStartupAnimation(elapsed, POWER_ON_HOLD_MS);
      nextRefresh += POWER_ON_DISPLAY_REFRESH_MS;
    }

    delay_ms(POWER_ON_SAMPLE_MS);
  }

  if (!accepted) {
    boardOff();
  }

  pwrOn();
  abnormalRebootResetCmd();
  drawStartupAnimation(POWER_ON_HOLD_MS, POWER_ON_HOLD_MS);

#if defined(HAPTIC)
  haptic.play(15, 3, PLAY_NOW);
#endif

  while (pwrPressed()) {
    WDG_RESET();
    delay_ms(POWER_ON_SAMPLE_MS);
  }
}
#endif

void boardOff()
{
#if defined(HELM_HAS_INTERNAL_FAN)
  fanSetPower(0);
#endif
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
#if defined(HELM_HOST_KEY_INJECTION) && defined(DEBUG_SEGGER_RTT)
  rttKeyInjectionPoll();
#endif
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
