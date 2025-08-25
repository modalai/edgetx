#pragma once

#include <stdint.h>
#include "hal.h"

#if defined(POWER_I2C)

void voxl_pm_init();

uint16_t voxl_pm_get_voltage();

#endif