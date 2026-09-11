/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#pragma once

#include <stdint.h>

#if defined(POWER_I2C)

void voxl_pm_init();
uint16_t voxl_pm_get_voltage();

#endif
