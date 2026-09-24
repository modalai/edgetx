/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#include "storage/helm_device_settings.h"

#include <stddef.h>
#include <string.h>

#include "edgetx.h"
#include "hal/adc_driver.h"
#include "hal/usb_driver.h"
#include "memory_sections.h"
#include "sdcard.h"
#include "targets/modalai/m0196/eeprom_93aa66.h"

namespace {

constexpr uint32_t RECORD_MAGIC = 0x4d4c4548;  // "HELM" in memory.
constexpr uint16_t RECORD_VERSION = 1;
constexpr uint16_t BOARD_PROFILE_HELM_BASIC = 1;
constexpr uint16_t RECORD_SIZE = 256;
constexpr uint16_t SLOT_ADDRESS[] = {0x000, 0x100};
constexpr uint8_t CALIBRATION_SLOTS = 12;

struct __attribute__((packed)) RecordHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t sequence;
  uint32_t crc;
  uint16_t boardProfile;
  uint8_t workflow;
  uint8_t flags;
  uint16_t calibrationMask;
  uint8_t calibrationCount;
  uint8_t reserved0;
  uint8_t reserved[8];
};

struct __attribute__((packed)) CoreSettings {
  uint8_t contrast;
  uint8_t invertLcd;
  uint8_t backlightMode;
  uint8_t backlightBright;
  uint8_t lightAutoOff;
  int8_t beepMode;
  int8_t beepLength;
  uint8_t speakerPitch;
  int8_t speakerVolume;
  int8_t beepVolume;
  int8_t wavVolume;
  int8_t backgroundVolume;
  uint8_t audioMuteEnable;
  uint8_t dontPlayHello;
  int8_t varioVolume;
  int8_t varioPitch;
  int8_t varioRange;
  int8_t varioRepeat;
  int8_t hapticMode;
  int8_t hapticLength;
  int8_t hapticStrength;
  uint8_t powerEventHaptics;
  int16_t timezoneMinutes;
  uint8_t disableRtcWarning;
  uint8_t vBatWarn;
  int8_t vBatMin;
  int8_t vBatMax;
  uint8_t inactivityTimer;
  uint8_t pwrOffIfInactive;
  int8_t splashMode;
  uint8_t bootMenu;
  uint8_t imperial;
  uint8_t gpsFormat;
  int8_t switchesDelay;
  uint8_t internalModuleBaudrate;
  uint8_t usbMode;
  uint8_t internalModuleDisabled;
  uint8_t reserved[10];
};

struct __attribute__((packed)) DeviceRecord {
  RecordHeader header;
  CalibData calibration[CALIBRATION_SLOTS];
  int8_t txVoltageCalibration;
  uint8_t hardwareReserved[23];
  CoreSettings core;
  uint32_t developerOptions[16];
  uint8_t futureReserved[16];
};

struct FactoryDefaults {
  CalibData calibration[CALIBRATION_SLOTS];
  int8_t txVoltageCalibration;
  alignas(4) CoreSettings core;
};

static_assert(sizeof(RecordHeader) == 32, "HELM record header size changed");
static_assert(sizeof(CoreSettings) == 48, "HELM core settings size changed");
static_assert(sizeof(DeviceRecord) == RECORD_SIZE,
              "HELM EEPROM record must fill one slot");
static_assert(offsetof(FactoryDefaults, core) % 4 == 0,
              "HELM factory settings cache must use aligned word access");

DeviceRecord activeRecord __DMA_NO_CACHE;
FactoryDefaults factoryDefaults __DMA_NO_CACHE;
bool haveActiveRecord;
bool haveFactoryDefaults;
uint8_t activeSlot;
bool internalModuleEnabled = true;
HelmDeviceSettingsStatus currentStatus = HelmDeviceSettingsStatus::Unused;

uint32_t crc32(const DeviceRecord& record)
{
  const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
  uint32_t crc = 0xffffffff;
  constexpr size_t crcOffset = offsetof(DeviceRecord, header.crc);

  for (size_t index = offsetof(DeviceRecord, header.version);
       index < sizeof(record); ++index) {
    uint8_t value = (index >= crcOffset && index < crcOffset + sizeof(uint32_t))
                        ? 0
                        : bytes[index];
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
  }
  return crc ^ 0xffffffff;
}

bool validWorkflow(uint8_t value)
{
  return value >= static_cast<uint8_t>(HelmDeviceWorkflow::InputTestRequired) &&
         value <= static_cast<uint8_t>(HelmDeviceWorkflow::Ready);
}

bool validRecord(const DeviceRecord& record)
{
  return record.header.magic == RECORD_MAGIC &&
         record.header.version == RECORD_VERSION &&
         record.header.length == sizeof(DeviceRecord) &&
         record.header.boardProfile == BOARD_PROFILE_HELM_BASIC &&
         record.header.calibrationCount <= CALIBRATION_SLOTS &&
         validWorkflow(record.header.workflow) && record.header.crc == crc32(record);
}

bool sequenceNewer(uint32_t first, uint32_t second)
{
  return static_cast<int32_t>(first - second) > 0;
}

CoreSettings captureCore()
{
  CoreSettings core = {};
  core.contrast = g_eeGeneral.contrast;
  core.invertLcd = g_eeGeneral.invertLCD;
  core.backlightMode = g_eeGeneral.backlightMode;
  core.backlightBright = g_eeGeneral.backlightBright;
  core.lightAutoOff = g_eeGeneral.lightAutoOff;
  core.beepMode = g_eeGeneral.beepMode;
  core.beepLength = g_eeGeneral.beepLength;
  core.speakerPitch = g_eeGeneral.speakerPitch;
  core.speakerVolume = g_eeGeneral.speakerVolume;
  core.beepVolume = g_eeGeneral.beepVolume;
  core.wavVolume = g_eeGeneral.wavVolume;
  core.backgroundVolume = g_eeGeneral.backgroundVolume;
  core.audioMuteEnable = g_eeGeneral.audioMuteEnable;
  core.dontPlayHello = g_eeGeneral.dontPlayHello;
  core.varioVolume = g_eeGeneral.varioVolume;
  core.varioPitch = g_eeGeneral.varioPitch;
  core.varioRange = g_eeGeneral.varioRange;
  core.varioRepeat = g_eeGeneral.varioRepeat;
  core.hapticMode = g_eeGeneral.hapticMode;
  core.hapticLength = g_eeGeneral.hapticLength;
  core.hapticStrength = g_eeGeneral.hapticStrength;
  core.powerEventHaptics = !g_eeGeneral.disablePwrOnOffHaptic;
  core.timezoneMinutes =
      static_cast<int16_t>(g_eeGeneral.timezone) * 60 +
      static_cast<int16_t>(g_eeGeneral.timezoneMinutes) * 15;
  core.disableRtcWarning = g_eeGeneral.disableRtcWarning;
  core.vBatWarn = g_eeGeneral.vBatWarn;
  core.vBatMin = g_eeGeneral.vBatMin;
  core.vBatMax = g_eeGeneral.vBatMax;
  core.inactivityTimer = g_eeGeneral.inactivityTimer;
  core.pwrOffIfInactive = g_eeGeneral.pwrOffIfInactive;
  core.splashMode = g_eeGeneral.splashMode;
  core.bootMenu = g_eeGeneral.bootMenu;
  core.imperial = g_eeGeneral.imperial;
  core.gpsFormat = g_eeGeneral.gpsFormat;
  core.switchesDelay = g_eeGeneral.switchesDelay;
  core.internalModuleBaudrate = g_eeGeneral.internalModuleBaudrate;
  core.usbMode = g_eeGeneral.USBMode;
  core.internalModuleDisabled = !internalModuleEnabled;
  return core;
}

void applyCore(const CoreSettings& core)
{
  g_eeGeneral.contrast = core.contrast;
  g_eeGeneral.invertLCD = core.invertLcd != 0;
  g_eeGeneral.backlightMode = core.backlightMode;
  g_eeGeneral.backlightBright = core.backlightBright;
  g_eeGeneral.lightAutoOff = core.lightAutoOff;
  g_eeGeneral.beepMode = core.beepMode;
  g_eeGeneral.beepLength = core.beepLength;
  g_eeGeneral.speakerPitch = core.speakerPitch;
  g_eeGeneral.speakerVolume = core.speakerVolume;
  g_eeGeneral.beepVolume = core.beepVolume;
  g_eeGeneral.wavVolume = core.wavVolume;
  g_eeGeneral.backgroundVolume = core.backgroundVolume;
  g_eeGeneral.audioMuteEnable = core.audioMuteEnable != 0;
  g_eeGeneral.dontPlayHello = core.dontPlayHello != 0;
  g_eeGeneral.varioVolume = core.varioVolume;
  g_eeGeneral.varioPitch = core.varioPitch;
  g_eeGeneral.varioRange = core.varioRange;
  g_eeGeneral.varioRepeat = core.varioRepeat;
  g_eeGeneral.hapticMode = core.hapticMode;
  g_eeGeneral.hapticLength = core.hapticLength;
  g_eeGeneral.hapticStrength = core.hapticStrength;
  g_eeGeneral.disablePwrOnOffHaptic = !core.powerEventHaptics;
  int16_t timezone = core.timezoneMinutes;
  if (timezone < -720) timezone = -720;
  if (timezone > 840) timezone = 840;
  g_eeGeneral.timezone = timezone / 60;
  g_eeGeneral.timezoneMinutes = (timezone % 60) / 15;
  g_eeGeneral.disableRtcWarning = core.disableRtcWarning != 0;
  g_eeGeneral.vBatWarn = core.vBatWarn;
  g_eeGeneral.vBatMin = core.vBatMin;
  g_eeGeneral.vBatMax = core.vBatMax;
  g_eeGeneral.inactivityTimer = core.inactivityTimer;
  g_eeGeneral.pwrOffIfInactive = core.pwrOffIfInactive;
  g_eeGeneral.splashMode = core.splashMode;
  g_eeGeneral.bootMenu = core.bootMenu;
  g_eeGeneral.imperial = core.imperial != 0;
  g_eeGeneral.gpsFormat = core.gpsFormat != 0;
  g_eeGeneral.switchesDelay = core.switchesDelay;
  g_eeGeneral.internalModuleBaudrate = core.internalModuleBaudrate;
  g_eeGeneral.USBMode = core.usbMode == USB_SERIAL_MODE
                            ? USB_SERIAL_MODE
                            : USB_JOYSTICK_MODE;
  internalModuleEnabled = core.internalModuleDisabled == 0;
}

DeviceRecord captureRecord(HelmDeviceWorkflow workflow, bool clearDeveloper)
{
  DeviceRecord record = {};
  record.header.magic = RECORD_MAGIC;
  record.header.version = RECORD_VERSION;
  record.header.length = sizeof(DeviceRecord);
  record.header.sequence = haveActiveRecord ? activeRecord.header.sequence + 1 : 1;
  record.header.boardProfile = BOARD_PROFILE_HELM_BASIC;
  record.header.workflow = static_cast<uint8_t>(workflow);
  record.header.calibrationCount = adcGetMaxCalibratedInputs();
  if (record.header.calibrationCount > CALIBRATION_SLOTS)
    record.header.calibrationCount = CALIBRATION_SLOTS;
  record.header.calibrationMask =
      record.header.calibrationCount == 0
          ? 0
          : static_cast<uint16_t>((1u << record.header.calibrationCount) - 1u);
  memcpy(record.calibration, g_eeGeneral.calib, sizeof(record.calibration));
  record.txVoltageCalibration = g_eeGeneral.txVoltageCalibration;
  record.core = captureCore();
  if (haveActiveRecord && !clearDeveloper) {
    memcpy(record.developerOptions, activeRecord.developerOptions,
           sizeof(record.developerOptions));
  }
  record.header.crc = crc32(record);
  return record;
}

bool commit(const DeviceRecord& desired)
{
  const uint8_t targetSlot = haveActiveRecord ? activeSlot ^ 1u : 0;
  DeviceRecord oldRecord;
  if (!helmEepromRead(SLOT_ADDRESS[targetSlot], &oldRecord, sizeof(oldRecord)))
    return false;

  const uint32_t invalidMagic = 0;
  if (!helmEepromWrite(SLOT_ADDRESS[targetSlot], &invalidMagic,
                       sizeof(invalidMagic))) {
    return false;
  }

  const auto* desiredBytes = reinterpret_cast<const uint8_t*>(&desired);
  const auto* oldBytes = reinterpret_cast<const uint8_t*>(&oldRecord);
  size_t offset = sizeof(desired.header.magic);
  while (offset < sizeof(desired)) {
    while (offset < sizeof(desired) && desiredBytes[offset] == oldBytes[offset])
      ++offset;
    if (offset == sizeof(desired)) break;
    const size_t start = offset;
    while (offset < sizeof(desired) && desiredBytes[offset] != oldBytes[offset])
      ++offset;
    if (!helmEepromWrite(SLOT_ADDRESS[targetSlot] + start,
                         desiredBytes + start, offset - start)) {
      return false;
    }
  }

  if (!helmEepromWrite(SLOT_ADDRESS[targetSlot], &desired.header.magic,
                       sizeof(desired.header.magic))) {
    return false;
  }

  DeviceRecord verify;
  if (!helmEepromRead(SLOT_ADDRESS[targetSlot], &verify, sizeof(verify)) ||
      memcmp(&verify, &desired, sizeof(verify)) != 0 || !validRecord(verify)) {
    return false;
  }

  activeRecord = desired;
  activeSlot = targetSlot;
  haveActiveRecord = true;
  return true;
}

HelmDeviceSettingsStatus statusForWorkflow(uint8_t workflow)
{
  switch (static_cast<HelmDeviceWorkflow>(workflow)) {
    case HelmDeviceWorkflow::InputTestRequired:
      return HelmDeviceSettingsStatus::InputTestRequired;
    case HelmDeviceWorkflow::CalibrationRequired:
      return HelmDeviceSettingsStatus::CalibrationRequired;
    case HelmDeviceWorkflow::Ready:
      return HelmDeviceSettingsStatus::Ready;
  }
  return HelmDeviceSettingsStatus::InputTestRequired;
}

HelmDeviceWorkflow currentWorkflow()
{
  if (!haveActiveRecord) return HelmDeviceWorkflow::InputTestRequired;
  return static_cast<HelmDeviceWorkflow>(activeRecord.header.workflow);
}

}  // namespace

HelmDeviceSettingsStatus helmDeviceSettingsLoadAndApply()
{
  if (!storageIsReadOnly()) {
    currentStatus = HelmDeviceSettingsStatus::Unused;
    return currentStatus;
  }

  memcpy(factoryDefaults.calibration, g_eeGeneral.calib,
         sizeof(factoryDefaults.calibration));
  factoryDefaults.txVoltageCalibration = g_eeGeneral.txVoltageCalibration;
  factoryDefaults.core = captureCore();
  haveFactoryDefaults = true;

  DeviceRecord records[2];
  if (!helmEepromRead(SLOT_ADDRESS[0], &records[0], sizeof(records[0])) ||
      !helmEepromRead(SLOT_ADDRESS[1], &records[1], sizeof(records[1]))) {
    currentStatus = HelmDeviceSettingsStatus::Fault;
    haveActiveRecord = false;
    return currentStatus;
  }

  const bool valid[] = {validRecord(records[0]), validRecord(records[1])};
  if (!valid[0] && !valid[1]) {
    currentStatus = HelmDeviceSettingsStatus::InputTestRequired;
    haveActiveRecord = false;
    return currentStatus;
  }

  activeSlot = valid[1] &&
                       (!valid[0] || sequenceNewer(records[1].header.sequence,
                                                  records[0].header.sequence))
                   ? 1
                   : 0;
  activeRecord = records[activeSlot];
  haveActiveRecord = true;
  applyCore(activeRecord.core);
  g_eeGeneral.txVoltageCalibration = activeRecord.txVoltageCalibration;

  currentStatus = statusForWorkflow(activeRecord.header.workflow);
  if (currentStatus == HelmDeviceSettingsStatus::Ready) {
    memcpy(g_eeGeneral.calib, activeRecord.calibration,
           sizeof(activeRecord.calibration));
    g_eeGeneral.chkSum = evalChkSum();
  }
  return currentStatus;
}

HelmDeviceSettingsStatus helmDeviceSettingsStatus()
{
  return currentStatus;
}

bool helmDeviceSettingsInternalModuleEnabled()
{
  return internalModuleEnabled;
}

void helmDeviceSettingsSetInternalModuleEnabled(bool enabled)
{
  internalModuleEnabled = enabled;
}

bool helmDeviceSettingsSetWorkflow(HelmDeviceWorkflow workflow)
{
  if (!storageIsReadOnly()) return true;
  const auto desired = captureRecord(workflow, false);
  if (!commit(desired)) return false;
  currentStatus = statusForWorkflow(static_cast<uint8_t>(workflow));
  return true;
}

bool helmDeviceSettingsSaveCurrent()
{
  if (!storageIsReadOnly()) return true;
  return helmDeviceSettingsSetWorkflow(currentWorkflow());
}

bool helmDeviceSettingsReset()
{
  if (!storageIsReadOnly()) return true;
  helmDeviceSettingsRestoreFactoryDefaults();
  const auto desired = captureRecord(HelmDeviceWorkflow::InputTestRequired, true);
  if (!commit(desired)) return false;
  currentStatus = HelmDeviceSettingsStatus::InputTestRequired;
  return true;
}

bool helmDeviceSettingsFinishCalibration()
{
  return helmDeviceSettingsSetWorkflow(HelmDeviceWorkflow::Ready);
}

void helmDeviceSettingsRestoreFactoryDefaults()
{
  if (!haveFactoryDefaults) return;
  memcpy(g_eeGeneral.calib, factoryDefaults.calibration,
         sizeof(factoryDefaults.calibration));
  g_eeGeneral.txVoltageCalibration = factoryDefaults.txVoltageCalibration;
  applyCore(factoryDefaults.core);
  g_eeGeneral.chkSum = evalChkSum();
}
