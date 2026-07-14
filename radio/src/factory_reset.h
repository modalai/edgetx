/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#pragma once

#include <stdint.h>

#include "ff.h"

enum class FactorySdEntryType : uint8_t {
  File,
  Directory,
};

struct FactorySdEntry {
  const char* path;
  uint32_t offset;
  uint32_t size;
  uint16_t checksum;
  FactorySdEntryType type;
};

extern const uint8_t factorySdData[];
extern const FactorySdEntry factorySdEntries[];
extern const uint16_t factorySdEntryCount;
extern const uint16_t factorySdFileCount;
extern const uint32_t factorySdDataSize;

enum class FactoryResetStage : uint8_t {
  None,
  Format,
  Mount,
  Prepare,
  Write,
  Verify,
  Marker,
  Reload,
};

struct FactoryResetResult {
  FactoryResetStage stage;
  FRESULT error;
  const char* path;

  bool ok() const { return error == FR_OK; }
};

enum class FactoryResetPendingState : uint8_t {
  None,
  Restoring,
  Test,
};

using FactoryResetProgress = void (*)(FactoryResetStage stage,
                                      uint16_t current,
                                      uint16_t total,
                                      const char* path);

FactoryResetPendingState factoryResetPendingState();
FactoryResetResult factoryResetRestoreSd(FactoryResetProgress progress);
FRESULT factoryResetClearPending();
bool factoryResetInputTestActive();
void factoryResetSetInputTestActive(bool active);
