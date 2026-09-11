/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include "edgetx.h"

#include <algorithm>
#include <stdio.h>

#include "factory_reset.h"
#include "gui/common/stdlcd/calibration.h"
#include "hal/adc_driver.h"
#include "hal/key_driver.h"
#include "hal/rotary_encoder.h"
#include "hal/switch_driver.h"
#include "storage/storage.h"

namespace {

enum class FactoryMenuState : uint8_t {
  Confirm,
  Error,
};

enum class TestPhase : uint8_t {
  SwitchBaseline,
  Keys,
  Trims,
  RotaryLeft,
  RotaryRight,
  Switches,
  Analogs,
  Summary,
};

struct InputTestState {
  TestPhase phase;
  uint8_t index;
  uint8_t passed;
  uint8_t expectedSwitchPosition;
  uint16_t analogMin;
  uint16_t analogMax;
  bool waitingForRelease;
  bool waitingForEnterRelease;
  rotenc_t rotaryStart;
};

FactoryMenuState factoryMenuState = FactoryMenuState::Confirm;
FactoryResetResult lastRestoreResult = {FactoryResetStage::None, FR_OK,
                                        nullptr};
InputTestState test = {};
bool calibrationSaveError = false;

const char* stageName(FactoryResetStage stage)
{
  switch (stage) {
    case FactoryResetStage::Format:
      return "Formatting SD";
    case FactoryResetStage::Mount:
      return "Mounting SD";
    case FactoryResetStage::Prepare:
      return "Creating folders";
    case FactoryResetStage::Write:
      return "Writing files";
    case FactoryResetStage::Verify:
      return "Verifying files";
    case FactoryResetStage::Marker:
      return "Finalizing";
    case FactoryResetStage::Reload:
      return "Loading defaults";
    default:
      return "Factory reset";
  }
}

void drawProgress(FactoryResetStage stage, uint16_t current, uint16_t total,
                  const char* path)
{
  lcdRefreshWait();
  lcdClear();
  title("Factory reset");
  lcdDrawText(LCD_W / 2, 2 * FH, stageName(stage), CENTERED);

  if (path) {
    const char* name = getBasename(path);
    lcdDrawSizedText(LCD_W / 2, 3 * FH, name, 20, CENTERED | SMLSIZE);
  }

  if (total > 0) {
    constexpr coord_t x = 9;
    constexpr coord_t width = LCD_W - 2 * x;
    lcdDrawRect(x, 5 * FH, width, 7);
    auto filled = static_cast<coord_t>(
        (static_cast<uint32_t>(width - 2) * current) / total);
    if (filled > 0) lcdDrawFilledRect(x + 1, 5 * FH + 1, filled, 5);
    lcdDrawNumber(LCD_W / 2 - 2, 6 * FH, current, RIGHT | SMLSIZE);
    lcdDrawChar(LCD_W / 2, 6 * FH, '/', SMLSIZE);
    lcdDrawNumber(LCD_W / 2 + 2, 6 * FH, total, LEFT | SMLSIZE);
  }

  lcdRefresh();
}

void resetInputTest()
{
  test = {};
  test.phase = TestPhase::SwitchBaseline;
  test.waitingForEnterRelease = (readKeys() & (1u << KEY_ENTER)) != 0;
  test.rotaryStart = rotaryEncoderGetValue();
}

void startInputTest()
{
  resetInputTest();
  chainMenu(menuFactoryInputTest);
}

void runRestore()
{
  lastRestoreResult = factoryResetRestoreSd(drawProgress);
  if (lastRestoreResult.ok()) {
    factoryMenuState = FactoryMenuState::Confirm;
    startInputTest();
  }
  else {
    factoryMenuState = FactoryMenuState::Error;
    AUDIO_ERROR();
  }
}

EnumKeys indexedKey(uint8_t index)
{
  auto supported = keysGetSupported();
  for (uint8_t key = 0; key < MAX_KEYS; ++key) {
    if ((supported & (1u << key)) && index-- == 0)
      return static_cast<EnumKeys>(key);
  }
  return KEY_MENU;
}

void showStepCount()
{
  lcdDrawNumber(LCD_W - 2, 0, test.passed + 1,
                RIGHT | INVERS | SMLSIZE);
}

void showCalibrationShortcut()
{
  lcdDrawText(LCD_W / 2, LCD_H - FH, "Hold ENT: calibrate",
              CENTERED | SMLSIZE);
}

void countPass()
{
  ++test.passed;
  test.waitingForRelease = false;
}

void startFactoryCalibration()
{
  calibrationSaveError = false;
  reusableBuffer.calib.state = CALIB_START;
  chainMenu(menuFactoryCalibration);
}

void enterSwitchPhase()
{
  test.phase = TestPhase::Switches;
  test.index = 0;
  test.expectedSwitchPosition = SWITCH_HW_DOWN;
}

void selectNextSwitch()
{
  if (test.index >= switchGetMaxSwitches()) {
    test.phase = TestPhase::Analogs;
    test.index = 0;
    test.analogMin = UINT16_MAX;
    test.analogMax = 0;
    return;
  }

  test.expectedSwitchPosition =
      switchGetHwType(test.index) == SWITCH_HW_3POS ? SWITCH_HW_MID
                                                    : SWITCH_HW_DOWN;
}

const char* switchPositionName(SwitchHwPos position)
{
  switch (position) {
    case SWITCH_HW_UP:
      return "UP";
    case SWITCH_HW_MID:
      return "MIDDLE";
    default:
      return "DOWN";
  }
}

uint8_t analogCount()
{
  return adcGetMaxInputs(ADC_INPUT_MAIN) + adcGetMaxInputs(ADC_INPUT_FLEX);
}

uint8_t analogRawIndex(uint8_t index)
{
  auto mainCount = adcGetMaxInputs(ADC_INPUT_MAIN);
  if (index < mainCount)
    return adcGetInputOffset(ADC_INPUT_MAIN) + index;
  return adcGetInputOffset(ADC_INPUT_FLEX) + index - mainCount;
}

const char* analogName(uint8_t index)
{
  auto mainCount = adcGetMaxInputs(ADC_INPUT_MAIN);
  if (index < mainCount) return adcGetInputName(ADC_INPUT_MAIN, index);
  return adcGetInputName(ADC_INPUT_FLEX, index - mainCount);
}

void finishAnalogStep()
{
  countPass();

  if (++test.index >= analogCount()) {
    test.phase = TestPhase::Summary;
  }
  else {
    test.analogMin = UINT16_MAX;
    test.analogMax = 0;
  }
}

void finishFactoryCalibration()
{
  storageCheck(true);
  if (storageDirtyMsk & EE_GENERAL) {
    calibrationSaveError = true;
    AUDIO_ERROR();
    return;
  }

  auto status = factoryResetClearPending();
  if (status != FR_OK) {
    calibrationSaveError = true;
    AUDIO_ERROR();
    return;
  }

  calibrationSaveError = false;
  menuCalibrationState = CALIB_START;
  menuLevel = 0;
  chainMenu(menuMainView);
}

}  // namespace

void menuFactoryReset(event_t event)
{
  if (event == EVT_ENTRY) factoryMenuState = FactoryMenuState::Confirm;

  title("Factory reset");
  if (factoryMenuState == FactoryMenuState::Confirm) {
    lcdDrawText(LCD_W / 2, 2 * FH, "ERASES THE SD CARD", CENTERED | INVERS);
    lcdDrawText(LCD_W / 2, 3 * FH, "Restores factory files", CENTERED);
    lcdDrawText(LCD_W / 2, 5 * FH, "Hold ENTER to start", CENTERED);
    lcdDrawText(LCD_W / 2, 6 * FH, "EXIT cancels", CENTERED | SMLSIZE);

    if (event == EVT_KEY_BREAK(KEY_EXIT)) {
      popMenu();
    }
    else if (event == EVT_KEY_LONG(KEY_ENTER)) {
      killEvents(event);
      runRestore();
    }
  }
  else {
    lcdDrawText(LCD_W / 2, 2 * FH, "RESTORE FAILED", CENTERED | INVERS);
    lcdDrawText(LCD_W / 2, 3 * FH, stageName(lastRestoreResult.stage),
                CENTERED);
    lcdDrawText(2, 4 * FH, "SD error:");
    lcdDrawNumber(lcdNextPos + 2, 4 * FH, lastRestoreResult.error, LEFT);
    if (lastRestoreResult.path) {
      lcdDrawSizedText(LCD_W / 2, 5 * FH,
                       getBasename(lastRestoreResult.path), 20,
                       CENTERED | SMLSIZE);
    }
    lcdDrawText(LCD_W / 2, 7 * FH, "Hold ENTER to retry",
                CENTERED | SMLSIZE);

    if (event == EVT_KEY_LONG(KEY_ENTER)) {
      killEvents(event);
      runRestore();
    }
  }
}

void menuFactoryResetResume(event_t event)
{
  title("Factory reset");
  lcdDrawText(LCD_W / 2, 2 * FH, "Restore interrupted", CENTERED | INVERS);
  lcdDrawText(LCD_W / 2, 4 * FH, "Resuming...", CENTERED);
  lcdRefresh();

  if (event == EVT_ENTRY) {
    factoryMenuState = FactoryMenuState::Error;
    runRestore();
  }
  else if (factoryMenuState == FactoryMenuState::Error) {
    menuFactoryReset(event);
  }
}

void menuFactoryInputTest(event_t event)
{
  if (event == EVT_ENTRY) {
    factoryResetSetInputTestActive(true);
    resetInputTest();
  }

  title("Control test");
  showStepCount();

  if (test.waitingForEnterRelease) {
    lcdDrawText(LCD_W / 2, 3 * FH, "Release ENTER", CENTERED | INVERS);
    if ((readKeys() & (1u << KEY_ENTER)) == 0) {
      test.waitingForEnterRelease = false;
      killEvents(KEY_ENTER);
    }
    return;
  }

  if (event == EVT_KEY_LONG(KEY_ENTER)) {
    killEvents(event);
    startFactoryCalibration();
    return;
  }

  switch (test.phase) {
    case TestPhase::SwitchBaseline: {
      lcdDrawText(LCD_W / 2, 2 * FH, "Set ALL switches UP", CENTERED);
      int badSwitch = -1;
      for (uint8_t i = 0; i < switchGetMaxSwitches(); ++i) {
        if (switchGetPosition(i) != SWITCH_HW_UP) {
          badSwitch = i;
          break;
        }
      }

      if (badSwitch < 0) {
        test.phase = TestPhase::Keys;
        test.index = 0;
      }
      else {
        lcdDrawText(LCD_W / 2, 4 * FH,
                    switchGetDefaultName(badSwitch), CENTERED | INVERS);
        lcdDrawText(LCD_W / 2, 5 * FH, "is not UP", CENTERED);
        showCalibrationShortcut();
      }
      break;
    }

    case TestPhase::Keys: {
      if (test.index >= keysGetMaxKeys()) {
        test.phase = TestPhase::Trims;
        test.index = 0;
        break;
      }

      auto key = indexedKey(test.index);
      lcdDrawText(LCD_W / 2, 2 * FH, "Press and release", CENTERED);
      lcdDrawText(LCD_W / 2, 4 * FH, keysGetLabel(key), CENTERED | DBLSIZE);
      showCalibrationShortcut();

      auto keys = readKeys();
      auto target = 1u << key;
      if (!test.waitingForRelease && keys == target) {
        test.waitingForRelease = true;
        // Keep ENTER events alive so a long press can end input testing.
        if (key != KEY_ENTER) killEvents(key);
      }
      else if (test.waitingForRelease && !(keys & target)) {
        countPass();
        ++test.index;
      }
      break;
    }

    case TestPhase::Trims: {
      auto trimDirections = keysGetMaxTrims() * 2;
      if (test.index >= trimDirections) {
        test.phase = TestPhase::RotaryLeft;
        test.rotaryStart = rotaryEncoderGetValue();
        break;
      }

      char label[20];
      snprintf(label, sizeof(label), "Trim %u %c", test.index / 2 + 1,
               (test.index & 1) ? '+' : '-');
      lcdDrawText(LCD_W / 2, 2 * FH, "Press and release", CENTERED);
      lcdDrawText(LCD_W / 2, 4 * FH, label, CENTERED | DBLSIZE);
      showCalibrationShortcut();

      auto trims = readTrims();
      auto target = 1u << test.index;
      if (!test.waitingForRelease && trims == target) {
        test.waitingForRelease = true;
      }
      else if (test.waitingForRelease && !(trims & target)) {
        countPass();
        ++test.index;
      }
      break;
    }

    case TestPhase::RotaryLeft:
      lcdDrawText(LCD_W / 2, 2 * FH, "Turn rotary LEFT", CENTERED | DBLSIZE);
      showCalibrationShortcut();
      if (rotaryEncoderGetValue() < test.rotaryStart) {
        test.phase = TestPhase::RotaryRight;
        test.rotaryStart = rotaryEncoderGetValue();
      }
      break;

    case TestPhase::RotaryRight:
      lcdDrawText(LCD_W / 2, 2 * FH, "Turn rotary RIGHT", CENTERED | DBLSIZE);
      showCalibrationShortcut();
      if (rotaryEncoderGetValue() > test.rotaryStart) {
        countPass();
        enterSwitchPhase();
        selectNextSwitch();
      }
      break;

    case TestPhase::Switches: {
      if (test.phase != TestPhase::Switches) break;

      auto expected = static_cast<SwitchHwPos>(test.expectedSwitchPosition);
      lcdDrawText(LCD_W / 2, 2 * FH, "Move switch", CENTERED);
      lcdDrawText(34, 4 * FH, switchGetDefaultName(test.index), DBLSIZE);
      lcdDrawText(75, 4 * FH, switchPositionName(expected), DBLSIZE);
      showCalibrationShortcut();

      if (switchGetPosition(test.index) == expected) {
        if (expected == SWITCH_HW_MID) {
          test.expectedSwitchPosition = SWITCH_HW_DOWN;
        }
        else if (expected == SWITCH_HW_DOWN) {
          test.expectedSwitchPosition = SWITCH_HW_UP;
        }
        else if (expected == SWITCH_HW_UP) {
          countPass();
          ++test.index;
          selectNextSwitch();
        }
      }
      break;
    }

    case TestPhase::Analogs: {
      if (test.index >= analogCount()) {
        test.phase = TestPhase::Summary;
        break;
      }

      auto value = getAnalogValue(analogRawIndex(test.index));
      test.analogMin = std::min(test.analogMin, value);
      test.analogMax = std::max(test.analogMax, value);
      auto span = test.analogMax - test.analogMin;

      lcdDrawText(LCD_W / 2, 2 * FH, "Move full range", CENTERED);
      lcdDrawText(LCD_W / 2, 3 * FH, analogName(test.index),
                  CENTERED | DBLSIZE);
      lcdDrawText(18, 5 * FH, "Range:");
      lcdDrawNumber(lcdNextPos + 2, 5 * FH, span, LEFT);
      showCalibrationShortcut();

      if (span >= (ADC_MAX_VALUE + 1) / 2) {
        finishAnalogStep();
      }
      break;
    }

    case TestPhase::Summary:
      lcdDrawText(LCD_W / 2, 2 * FH, "Control test complete", CENTERED | INVERS);
      lcdDrawText(24, 4 * FH, "Passed:");
      lcdDrawNumber(lcdNextPos + 3, 4 * FH, test.passed, LEFT);
      lcdDrawText(LCD_W / 2, 7 * FH, "ENTER: calibrate",
                  CENTERED | SMLSIZE);
      if (event == EVT_KEY_BREAK(KEY_ENTER)) {
        startFactoryCalibration();
      }
      break;
  }
}

void menuFactoryCalibration(event_t event)
{
  if (event == EVT_ENTRY) factoryResetSetInputTestActive(false);

  if (calibrationSaveError) {
    title("Factory reset");
    lcdDrawText(LCD_W / 2, 2 * FH, "SAVE FAILED", CENTERED | INVERS);
    lcdDrawText(LCD_W / 2, 4 * FH, "Check SD card", CENTERED);
    lcdDrawText(LCD_W / 2, 6 * FH, "Hold ENTER to retry",
                CENTERED | SMLSIZE);
    if (event == EVT_KEY_LONG(KEY_ENTER)) {
      killEvents(event);
      finishFactoryCalibration();
    }
    return;
  }

  if (reusableBuffer.calib.state == CALIB_FINISHED) {
    finishFactoryCalibration();
    return;
  }

  lcdDrawText(LCD_W / 2, 0, "Mandatory calibration", CENTERED);
  lcdInvertLine(0);
  menuCommonCalib(event == EVT_KEY_BREAK(KEY_EXIT) ? 0 : event);
}
