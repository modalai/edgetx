/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#include "storage/factory_volume.h"

#include <string.h>

#include "FatFs/diskio.h"
#include "memory_sections.h"

namespace {

constexpr uint32_t SECTOR_SIZE = 512;
FATFS fileSystem __CCMRAM;

DSTATUS initialize(BYTE)
{
  return factorySdImageSize != 0 &&
                 (factorySdImageSize % SECTOR_SIZE) == 0
             ? STA_PROTECT
             : STA_NOINIT;
}

DSTATUS status(BYTE)
{
  return initialize(0);
}

DRESULT read(BYTE, BYTE* buffer, DWORD sector, UINT count)
{
  const uint32_t offset = sector * SECTOR_SIZE;
  const uint32_t size = count * SECTOR_SIZE;
  if (!buffer || count == 0 || offset > factorySdImageSize ||
      size > factorySdImageSize - offset) {
    return RES_PARERR;
  }

  memcpy(buffer, factorySdImage + offset, size);
  return RES_OK;
}

DRESULT write(BYTE, const BYTE*, DWORD, UINT)
{
  return RES_WRPRT;
}

DRESULT ioctl(BYTE, BYTE command, void* buffer)
{
  switch (command) {
    case CTRL_SYNC:
      return RES_OK;
    case GET_SECTOR_COUNT:
      if (!buffer) return RES_PARERR;
      *static_cast<DWORD*>(buffer) = factorySdImageSize / SECTOR_SIZE;
      return RES_OK;
    case GET_SECTOR_SIZE:
      if (!buffer) return RES_PARERR;
      *static_cast<WORD*>(buffer) = SECTOR_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      if (!buffer) return RES_PARERR;
      *static_cast<DWORD*>(buffer) = 1;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

}  // namespace

const diskio_driver_t factoryVolumeDiskioDriver = {
  .initialize = initialize,
  .deinit = nullptr,
  .status = status,
  .read = read,
  .write = write,
  .ioctl = ioctl,
};

FATFS* factoryVolumeFileSystem()
{
  return &fileSystem;
}
