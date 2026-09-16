/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: This program is free software. You can redistribute it
 * and modify it under the terms of the GNU General Public License, version 2.
 */

#include "factory_reset.h"

#include <algorithm>
#include <stdio.h>
#include <string.h>

#include "edgetx.h"
#include "hal/storage.h"
#include "hal/watchdog_driver.h"
#include "memory_sections.h"
#include "sdcard.h"
#include "storage/factory_volume.h"
#include "storage/helm_device_settings.h"

namespace {

constexpr char FACTORY_RESET_MARKER[] = "0:" RADIO_PATH "/.factory-reset-pending";
constexpr char MARKER_RESTORING[] = "RESTORING\n";
constexpr char MARKER_TEST[] = "TEST\n";
constexpr uint16_t IO_BUFFER_SIZE = 4096;
constexpr size_t PATH_BUFFER_SIZE = 512;

uint8_t formatWork[FF_MAX_SS] __DMA;
uint8_t ioBuffer[IO_BUFFER_SIZE] __DMA;
uint8_t verifyBuffer[IO_BUFFER_SIZE] __DMA;
bool runtimeSuspended = false;
bool inputTestActive = false;
uint16_t copiedFiles;

FactoryResetResult result(FactoryResetStage stage, FRESULT error,
                          const char* path = nullptr)
{
  return {stage, error, path};
}

void report(FactoryResetProgress progress, FactoryResetStage stage,
            uint16_t current, uint16_t total, const char* path = nullptr)
{
  watchdogSuspend(6000);
  if (progress) progress(stage, current, total, path);
}

FRESULT writeMarker(const char* value)
{
  FIL file = {};
  auto status = f_open(&file, FACTORY_RESET_MARKER,
                       FA_CREATE_ALWAYS | FA_WRITE);
  if (status != FR_OK) return status;

  UINT written = 0;
  const auto length = static_cast<UINT>(strlen(value));
  status = f_write(&file, value, length, &written);
  if (status == FR_OK && written != length) status = FR_DISK_ERR;
  if (status == FR_OK) status = f_sync(&file);
  const auto closeStatus = f_close(&file);
  return status == FR_OK ? closeStatus : status;
}

FRESULT createDirectory(const char* path)
{
  const auto status = f_mkdir(path);
  return status == FR_EXIST ? FR_OK : status;
}

bool makePath(char* output, size_t size, const char* drive,
              const char* relative)
{
  const int length = snprintf(output, size, "%s%s", drive, relative);
  return length >= 0 && static_cast<size_t>(length) < size;
}

bool appendPath(char* output, size_t size, const char* parent,
                const char* name)
{
  const int length = strcmp(parent, "/") == 0
                         ? snprintf(output, size, "/%s", name)
                         : snprintf(output, size, "%s/%s", parent, name);
  return length >= 0 && static_cast<size_t>(length) < size;
}

FRESULT copyFile(const char* relative, FactoryResetProgress progress)
{
  char sourcePath[PATH_BUFFER_SIZE];
  char destinationPath[PATH_BUFFER_SIZE];
  if (!makePath(sourcePath, sizeof(sourcePath), "1:", relative) ||
      !makePath(destinationPath, sizeof(destinationPath), "0:", relative)) {
    return FR_INVALID_NAME;
  }

  report(progress, FactoryResetStage::Write, ++copiedFiles,
         factorySdFileCount, relative);

  FIL source = {};
  FIL destination = {};
  auto status = f_open(&source, sourcePath, FA_READ);
  if (status != FR_OK) return status;
  status = f_open(&destination, destinationPath, FA_CREATE_ALWAYS | FA_WRITE);
  if (status != FR_OK) {
    f_close(&source);
    return status;
  }

  while (status == FR_OK) {
    UINT bytesRead = 0;
    UINT bytesWritten = 0;
    status = f_read(&source, ioBuffer, sizeof(ioBuffer), &bytesRead);
    if (status != FR_OK || bytesRead == 0) break;
    status = f_write(&destination, ioBuffer, bytesRead, &bytesWritten);
    if (status == FR_OK && bytesWritten != bytesRead) status = FR_DISK_ERR;
    watchdogSuspend(6000);
  }

  if (status == FR_OK) status = f_sync(&destination);
  const auto sourceClose = f_close(&source);
  const auto destinationClose = f_close(&destination);
  if (status == FR_OK) status = sourceClose;
  if (status == FR_OK) status = destinationClose;
  return status;
}

FRESULT copyDirectory(const char* relative, FactoryResetProgress progress)
{
  char sourcePath[PATH_BUFFER_SIZE];
  char destinationPath[PATH_BUFFER_SIZE];
  if (!makePath(sourcePath, sizeof(sourcePath), "1:", relative) ||
      !makePath(destinationPath, sizeof(destinationPath), "0:", relative)) {
    return FR_INVALID_NAME;
  }

  if (strcmp(relative, "/") != 0) {
    auto status = createDirectory(destinationPath);
    if (status != FR_OK) return status;
  }

  DIR directory = {};
  auto status = f_opendir(&directory, sourcePath);
  if (status != FR_OK) return status;

  FILINFO info = {};
  while ((status = f_readdir(&directory, &info)) == FR_OK && info.fname[0]) {
    if (strcmp(info.fname, ".") == 0 || strcmp(info.fname, "..") == 0)
      continue;

    char child[PATH_BUFFER_SIZE];
    if (!appendPath(child, sizeof(child), relative, info.fname)) {
      status = FR_INVALID_NAME;
      break;
    }
    status = (info.fattrib & AM_DIR) ? copyDirectory(child, progress)
                                     : copyFile(child, progress);
    if (status != FR_OK) break;
  }

  const auto closeStatus = f_closedir(&directory);
  return status == FR_OK ? closeStatus : status;
}

FRESULT verifyFile(const char* relative, FactoryResetProgress progress,
                   uint16_t& verifiedFiles)
{
  char sourcePath[PATH_BUFFER_SIZE];
  char destinationPath[PATH_BUFFER_SIZE];
  if (!makePath(sourcePath, sizeof(sourcePath), "1:", relative) ||
      !makePath(destinationPath, sizeof(destinationPath), "0:", relative)) {
    return FR_INVALID_NAME;
  }

  report(progress, FactoryResetStage::Verify, ++verifiedFiles,
         factorySdFileCount, relative);

  FIL source = {};
  FIL destination = {};
  auto status = f_open(&source, sourcePath, FA_READ);
  if (status != FR_OK) return status;
  status = f_open(&destination, destinationPath, FA_READ);
  if (status != FR_OK) {
    f_close(&source);
    return status;
  }

  if (f_size(&source) != f_size(&destination)) status = FR_INT_ERR;
  while (status == FR_OK) {
    UINT sourceRead = 0;
    UINT destinationRead = 0;
    status = f_read(&source, ioBuffer, sizeof(ioBuffer), &sourceRead);
    if (status != FR_OK) break;
    status = f_read(&destination, verifyBuffer, sizeof(verifyBuffer),
                    &destinationRead);
    if (status != FR_OK) break;
    if (sourceRead != destinationRead ||
        memcmp(ioBuffer, verifyBuffer, sourceRead) != 0) {
      status = FR_INT_ERR;
      break;
    }
    if (sourceRead == 0) break;
    watchdogSuspend(6000);
  }

  const auto sourceClose = f_close(&source);
  const auto destinationClose = f_close(&destination);
  if (status == FR_OK) status = sourceClose;
  if (status == FR_OK) status = destinationClose;
  return status;
}

FRESULT verifyDirectory(const char* relative, FactoryResetProgress progress,
                        uint16_t& verifiedFiles)
{
  char sourcePath[PATH_BUFFER_SIZE];
  if (!makePath(sourcePath, sizeof(sourcePath), "1:", relative))
    return FR_INVALID_NAME;

  DIR directory = {};
  auto status = f_opendir(&directory, sourcePath);
  if (status != FR_OK) return status;

  FILINFO info = {};
  while ((status = f_readdir(&directory, &info)) == FR_OK && info.fname[0]) {
    if (strcmp(info.fname, ".") == 0 || strcmp(info.fname, "..") == 0)
      continue;
    char child[PATH_BUFFER_SIZE];
    if (!appendPath(child, sizeof(child), relative, info.fname)) {
      status = FR_INVALID_NAME;
      break;
    }
    status = (info.fattrib & AM_DIR)
                 ? verifyDirectory(child, progress, verifiedFiles)
                 : verifyFile(child, progress, verifiedFiles);
    if (status != FR_OK) break;
  }

  const auto closeStatus = f_closedir(&directory);
  return status == FR_OK ? closeStatus : status;
}

void cleanUpFailedRestore()
{
  f_chdrive("0:");
  sdDone();
  watchdogSuspend(0);
}

}  // namespace

FactoryResetPendingState factoryResetPendingState()
{
  if (storageIsReadOnly()) {
    switch (helmDeviceSettingsStatus()) {
      case HelmDeviceSettingsStatus::InputTestRequired:
        return FactoryResetPendingState::Test;
      case HelmDeviceSettingsStatus::CalibrationRequired:
        return FactoryResetPendingState::Calibration;
      default:
        return FactoryResetPendingState::None;
    }
  }

  FIL file = {};
  if (f_open(&file, FACTORY_RESET_MARKER, FA_READ) != FR_OK)
    return FactoryResetPendingState::None;

  char marker[sizeof(MARKER_RESTORING)] = {};
  UINT bytesRead = 0;
  auto status = f_read(&file, marker, sizeof(marker) - 1, &bytesRead);
  const auto closeStatus = f_close(&file);
  if (status != FR_OK || closeStatus != FR_OK)
    return FactoryResetPendingState::Restoring;
  if (bytesRead == strlen(MARKER_TEST) &&
      memcmp(marker, MARKER_TEST, bytesRead) == 0)
    return FactoryResetPendingState::Test;
  return FactoryResetPendingState::Restoring;
}

FactoryResetResult factoryResetRestoreSd(FactoryResetProgress progress)
{
  if (!storageIsPresent()) return result(FactoryResetStage::Mount, FR_NOT_READY);

  if (!runtimeSuspended) {
    edgeTxClose(false);
    runtimeSuspended = true;
  }

  report(progress, FactoryResetStage::Format, 0, 1);
  storageInit();
  MKFS_PARM options = {};
  options.fmt = FM_FAT32;
  auto status = f_mkfs("0:", &options, formatWork, sizeof(formatWork));
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Format, status);
  }

  report(progress, FactoryResetStage::Mount, 0, 1);
  sdMount();
  if (!sdMounted() ||
      f_mount(factoryVolumeFileSystem(), "1:", 1) != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Mount, FR_NOT_READY);
  }

  report(progress, FactoryResetStage::Prepare, 0, factorySdFileCount);
  status = createDirectory("0:" RADIO_PATH);
  if (status == FR_OK) status = writeMarker(MARKER_RESTORING);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Prepare, status, RADIO_PATH);
  }

  copiedFiles = 0;
  status = copyDirectory("/", progress);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Write, status);
  }

  uint16_t verifiedFiles = 0;
  status = verifyDirectory("/", progress, verifiedFiles);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Verify, status);
  }

  report(progress, FactoryResetStage::Marker, 1, 1);
  status = writeMarker(MARKER_TEST);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Marker, status, FACTORY_RESET_MARKER);
  }

  report(progress, FactoryResetStage::Reload, 1, 1);
  f_chdrive("0:");
  sdDone();
  edgeTxResume();
  runtimeSuspended = false;
  watchdogSuspend(0);
  return result(FactoryResetStage::None, FR_OK);
}

bool factoryResetBeginWithoutSd()
{
  return storageIsReadOnly() && helmDeviceSettingsReset();
}

FRESULT factoryResetClearPending()
{
  if (storageIsReadOnly()) return FR_OK;
  const auto status = f_unlink(FACTORY_RESET_MARKER);
  if (status == FR_NO_FILE || status == FR_NO_PATH) return FR_OK;
  return status;
}

bool factoryResetInputTestActive()
{
  return inputTestActive;
}

void factoryResetSetInputTestActive(bool active)
{
  inputTestActive = active;
}
