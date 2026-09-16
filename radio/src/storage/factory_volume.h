/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#pragma once

#include <stdint.h>

#include "ff.h"
#include "hal/fatfs_diskio.h"

extern const uint8_t factorySdImage[];
extern const uint32_t factorySdImageSize;
extern const uint16_t factorySdFileCount;
extern const diskio_driver_t factoryVolumeDiskioDriver;

FATFS* factoryVolumeFileSystem();

constexpr uint8_t FACTORY_VOLUME_DRIVE = 1;
constexpr const char* FACTORY_VOLUME_PATH = "1:";
