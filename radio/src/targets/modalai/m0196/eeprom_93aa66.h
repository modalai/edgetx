/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr uint16_t HELM_EEPROM_SIZE = 512;

void helmEepromInit();
bool helmEepromRead(uint16_t address, void* data, size_t size);
bool helmEepromWrite(uint16_t address, const void* data, size_t size);
