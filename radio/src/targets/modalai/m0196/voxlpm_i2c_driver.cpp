/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#include "voxlpm_i2c_driver.h"

#include "hal/i2c_driver.h"
#include "stm32_i2c_driver.h"

#include "hal.h"

#if defined(POWER_I2C)

namespace {

constexpr uint8_t INA231_CONFIG_REGISTER = 0x00;
constexpr uint8_t INA231_BUS_VOLTAGE_REGISTER = 0x02;
// Preserve reserved D14. Select 16 samples per average, 1.1 ms conversions,
// and continuous shunt-plus-bus mode.
constexpr uint16_t INA231_CONFIG = 0x4527;
constexpr uint8_t INA231_REGISTER_ADDRESS_SIZE = 1;
constexpr uint8_t INA231_REGISTER_SIZE = 2;
constexpr uint8_t INA231_BUS_VOLTAGE_DIVISOR = 8;

bool busInitialized;
bool monitorConfigured;

bool configureMonitor()
{
  uint8_t config[INA231_REGISTER_SIZE] = {
    static_cast<uint8_t>(INA231_CONFIG >> 8),
    static_cast<uint8_t>(INA231_CONFIG),
  };

  if (i2c_write(I2C_Bus_1, HELM_POWER_MONITOR_ADDRESS,
                INA231_CONFIG_REGISTER, INA231_REGISTER_ADDRESS_SIZE, config,
                sizeof(config)) < 0) {
    return false;
  }

  monitorConfigured = true;
  return true;
}

bool initializeMonitor()
{
  if (!busInitialized) {
    if (i2c_init(I2C_Bus_1) < 0) {
      return false;
    }
    busInitialized = true;
  }

  return monitorConfigured || configureMonitor();
}

}  // namespace

void voxl_pm_init()
{
  initializeMonitor();
}

uint16_t voxl_pm_get_voltage()
{
  if (!initializeMonitor()) {
    return 0;
  }

  uint8_t rawData[INA231_REGISTER_SIZE];
  if (i2c_read(I2C_Bus_1, HELM_POWER_MONITOR_ADDRESS,
               INA231_BUS_VOLTAGE_REGISTER, INA231_REGISTER_ADDRESS_SIZE,
               rawData, sizeof(rawData)) < 0) {
    return 0;
  }

  uint16_t rawVoltage =
      (static_cast<uint16_t>(rawData[0]) << 8) | rawData[1];

  // Each register bit is 1.25 mV. EdgeTX uses units of 10 mV.
  return rawVoltage / INA231_BUS_VOLTAGE_DIVISOR;
}

#endif
