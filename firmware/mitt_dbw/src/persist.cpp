// Copyright 2026 Jaerock Kwon
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "persist.h"

#include <Arduino.h>
#include <EEPROM.h>

#include "config.h"
#include "hw.h"

namespace persist {

namespace {

constexpr uint32_t kMagic = 0x4D445657;  // "MDVW"
constexpr uint16_t kLayout = 1;          // bump when Params changes shape
constexpr int kAddr = 0;

struct Record {
  uint32_t magic;
  uint16_t layout;
  uint16_t size;
  dbw::Params params;
  uint32_t crc;
};

uint32_t Crc32(const uint8_t *data, size_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

uint32_t RecordCrc(const Record &r) {
  return Crc32(reinterpret_cast<const uint8_t *>(&r), offsetof(Record, crc));
}

}  // namespace

bool Load(dbw::Params *out) {
  Record r;
  EEPROM.get(kAddr, r);
  if (r.magic != kMagic || r.layout != kLayout || r.size != sizeof(dbw::Params)) return false;
  if (r.crc != RecordCrc(r)) return false;
  *out = r.params;
  return true;
}

void Save(const dbw::Params &p) {
  Record r;
  memset(static_cast<void *>(&r), 0, sizeof(r));  // any padding too: a reproducible CRC
  r.magic = kMagic;
  r.layout = kLayout;
  r.size = sizeof(dbw::Params);
  r.params = p;
  r.crc = RecordCrc(r);

  // Flash erase runs with interrupts off, so the control tick - the only
  // thing that feeds the watchdog - cannot run. Stretch the watchdog for the
  // write and put it back. The pulse timers keep emitting what they last
  // had, which the caller has made neutral.
  hw::WatchdogSetTimeout(cfg::kWatchdogSaveMs);
  EEPROM.put(kAddr, r);
  hw::WatchdogSetTimeout(cfg::kWatchdogMs);
}

}  // namespace persist
