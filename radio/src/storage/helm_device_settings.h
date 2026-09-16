/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#pragma once

#include <stdint.h>

enum class HelmDeviceWorkflow : uint8_t {
  InputTestRequired = 1,
  CalibrationRequired = 2,
  Ready = 3,
};

enum class HelmDeviceSettingsStatus : uint8_t {
  Unused,
  Ready,
  InputTestRequired,
  CalibrationRequired,
  Fault,
};

HelmDeviceSettingsStatus helmDeviceSettingsLoadAndApply();
HelmDeviceSettingsStatus helmDeviceSettingsStatus();
bool helmDeviceSettingsSetWorkflow(HelmDeviceWorkflow workflow);
bool helmDeviceSettingsSaveCurrent();
bool helmDeviceSettingsReset();
bool helmDeviceSettingsFinishCalibration();
void helmDeviceSettingsRestoreFactoryDefaults();
