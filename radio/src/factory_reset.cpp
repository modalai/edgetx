/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include "factory_reset.h"

#include <algorithm>
#include <string.h>

#include "crc.h"
#include "edgetx.h"
#include "hal/storage.h"
#include "hal/watchdog_driver.h"
#include "memory_sections.h"
#include "sdcard.h"

namespace {

constexpr char FACTORY_RESET_MARKER[] = RADIO_PATH "/.factory-reset-pending";
constexpr char MARKER_RESTORING[] = "RESTORING\n";
constexpr char MARKER_TEST[] = "TEST\n";
constexpr uint16_t IO_BUFFER_SIZE = 4096;

uint8_t formatWork[FF_MAX_SS] __DMA;
uint8_t ioBuffer[IO_BUFFER_SIZE] __DMA;
bool runtimeSuspended = false;
bool inputTestActive = false;

FactoryResetResult result(FactoryResetStage stage, FRESULT error,
                          const char* path = nullptr)
{
  return {stage, error, path};
}

void report(FactoryResetProgress progress, FactoryResetStage stage,
            uint16_t current, uint16_t total, const char* path = nullptr)
{
  watchdogSuspend(6000);  // Keep a one-minute margin during blocking I/O.
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

  auto closeStatus = f_close(&file);
  return status == FR_OK ? closeStatus : status;
}

FRESULT createDirectory(const char* path)
{
  auto status = f_mkdir(path);
  return status == FR_EXIST ? FR_OK : status;
}

FRESULT writeEntry(const FactorySdEntry& entry)
{
  FIL file = {};
  auto status = f_open(&file, entry.path, FA_CREATE_ALWAYS | FA_WRITE);
  if (status != FR_OK) return status;

  uint32_t position = 0;
  while (position < entry.size) {
    const auto count = static_cast<UINT>(
        std::min<uint32_t>(IO_BUFFER_SIZE, entry.size - position));
    UINT written = 0;
    status = f_write(&file, factorySdData + entry.offset + position, count,
                     &written);
    if (status != FR_OK || written != count) {
      if (status == FR_OK) status = FR_DISK_ERR;
      break;
    }
    position += written;
    watchdogSuspend(6000);
  }

  if (status == FR_OK) status = f_sync(&file);
  auto closeStatus = f_close(&file);
  return status == FR_OK ? closeStatus : status;
}

FRESULT verifyEntry(const FactorySdEntry& entry)
{
  FILINFO info = {};
  auto status = f_stat(entry.path, &info);
  if (status != FR_OK) return status;
  if ((info.fattrib & AM_DIR) || info.fsize != entry.size) return FR_INT_ERR;

  FIL file = {};
  status = f_open(&file, entry.path, FA_READ);
  if (status != FR_OK) return status;

  uint32_t remaining = entry.size;
  uint16_t checksum = 0;
  while (remaining > 0) {
    const auto count =
        static_cast<UINT>(std::min<uint32_t>(IO_BUFFER_SIZE, remaining));
    UINT bytesRead = 0;
    status = f_read(&file, ioBuffer, count, &bytesRead);
    if (status != FR_OK || bytesRead != count) {
      if (status == FR_OK) status = FR_DISK_ERR;
      break;
    }
    checksum = crc16(CRC_1021, ioBuffer, bytesRead, checksum);
    remaining -= bytesRead;
    watchdogSuspend(6000);
  }

  auto closeStatus = f_close(&file);
  if (status == FR_OK) status = closeStatus;
  if (status == FR_OK && checksum != entry.checksum) status = FR_INT_ERR;
  return status;
}

void cleanUpFailedRestore()
{
  f_chdir("/");
  sdDone();
  watchdogSuspend(0);
}

}  // namespace

FactoryResetPendingState factoryResetPendingState()
{
  FIL file = {};
  if (f_open(&file, FACTORY_RESET_MARKER, FA_READ) != FR_OK)
    return FactoryResetPendingState::None;

  char marker[sizeof(MARKER_RESTORING)] = {};
  UINT bytesRead = 0;
  auto status = f_read(&file, marker, sizeof(marker) - 1, &bytesRead);
  auto closeStatus = f_close(&file);
  if (status != FR_OK || closeStatus != FR_OK)
    return FactoryResetPendingState::Restoring;

  if (bytesRead == strlen(MARKER_TEST) &&
      memcmp(marker, MARKER_TEST, bytesRead) == 0)
    return FactoryResetPendingState::Test;

  return FactoryResetPendingState::Restoring;
}

FactoryResetResult factoryResetRestoreSd(FactoryResetProgress progress)
{
  if (!runtimeSuspended) {
    edgeTxClose(false);
    runtimeSuspended = true;
  }

  report(progress, FactoryResetStage::Format, 0, 1);
  storageInit();

  MKFS_PARM options = {};
  options.fmt = FM_FAT32;
  auto status = f_mkfs("", &options, formatWork, sizeof(formatWork));
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Format, status);
  }

  report(progress, FactoryResetStage::Mount, 0, 1);
  sdMount();
  if (!sdMounted()) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Mount, FR_NOT_READY);
  }
  f_chdir("/");

  report(progress, FactoryResetStage::Prepare, 0, factorySdEntryCount);
  status = createDirectory(RADIO_PATH);
  if (status == FR_OK) status = writeMarker(MARKER_RESTORING);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Prepare, status, RADIO_PATH);
  }

  for (uint16_t i = 0; i < factorySdEntryCount; ++i) {
    const auto& entry = factorySdEntries[i];
    if (entry.type != FactorySdEntryType::Directory) continue;
    report(progress, FactoryResetStage::Prepare, i + 1, factorySdEntryCount,
           entry.path);
    status = createDirectory(entry.path);
    if (status != FR_OK) {
      cleanUpFailedRestore();
      return result(FactoryResetStage::Prepare, status, entry.path);
    }
  }

  uint16_t fileIndex = 0;
  for (uint16_t i = 0; i < factorySdEntryCount; ++i) {
    const auto& entry = factorySdEntries[i];
    if (entry.type != FactorySdEntryType::File) continue;
    report(progress, FactoryResetStage::Write, ++fileIndex,
           factorySdFileCount, entry.path);
    status = writeEntry(entry);
    if (status != FR_OK) {
      cleanUpFailedRestore();
      return result(FactoryResetStage::Write, status, entry.path);
    }
  }

  fileIndex = 0;
  for (uint16_t i = 0; i < factorySdEntryCount; ++i) {
    const auto& entry = factorySdEntries[i];
    if (entry.type != FactorySdEntryType::File) continue;
    report(progress, FactoryResetStage::Verify, ++fileIndex,
           factorySdFileCount, entry.path);
    status = verifyEntry(entry);
    if (status != FR_OK) {
      cleanUpFailedRestore();
      return result(FactoryResetStage::Verify, status, entry.path);
    }
  }

  report(progress, FactoryResetStage::Marker, 1, 1);
  status = writeMarker(MARKER_TEST);
  if (status != FR_OK) {
    cleanUpFailedRestore();
    return result(FactoryResetStage::Marker, status, FACTORY_RESET_MARKER);
  }

  report(progress, FactoryResetStage::Reload, 1, 1);
  f_chdir("/");
  sdDone();
  edgeTxResume();
  runtimeSuspended = false;
  watchdogSuspend(0);
  return result(FactoryResetStage::None, FR_OK);
}

FRESULT factoryResetClearPending()
{
  auto status = f_unlink(FACTORY_RESET_MARKER);
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
