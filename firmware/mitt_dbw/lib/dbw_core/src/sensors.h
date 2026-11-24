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

#pragma once

#include <stdint.h>

#include "dbw_types.h"

namespace dbw {

// AS5600 counts -> road-wheel radians, and the judgement of whether to
// believe them. safety.md row 9: "Never run the loop on a bad angle."
//
// The angle is k * wrap(raw - c0), with the difference wrapped to
// -2048..2047. So the sensor's own 4095 -> 0 rollover is harmless anywhere,
// and the only discontinuity sits half a sensor-turn from centre - which the
// bench wrap check (firmware/bench as5600_check) proves is never reached.
class AngleSensor {
 public:
  struct Sample {
    bool read_ok = false;    // both I2C transactions completed
    bool magnet_ok = false;  // STATUS: MD set, ML and MH clear
    uint16_t raw = 0;        // RAW_ANGLE, 0..4095
  };

  void Update(const Sample &s, float dt_s, const Params &p);

  bool ok() const { return ok_; }
  float angle_rad() const { return angle_; }
  float rate_rad_s() const { return rate_; }
  uint16_t raw() const { return raw_; }

  // Counts relative to c0 (or to 0 when never zeroed) - for calibration.
  static int32_t Wrap(int32_t counts);

 private:
  bool ok_ = false;
  bool have_last_ = false;
  int good_streak_ = 0;
  float angle_ = 0.0f;
  float last_angle_ = 0.0f;
  float rate_ = 0.0f;
  uint16_t raw_ = 0;
};

// Wheel speed from cumulative drive ticks, over a sliding window. At 36
// counts per wheel turn the counts are sparse - about 45 a second at 1 m/s -
// so a per-tick difference is mostly zero. 200 ms of history gives about
// 0.1 m/s resolution, enough for the direction interlock and telemetry.
// Odometry does not use this: it integrates DbwStatus.drive_ticks.
class WheelSpeed {
 public:
  static constexpr int kSlots = 64;
  static constexpr uint32_t kWindowMs = 200;

  void Update(int32_t ticks, uint32_t now_ms, float m_per_tick);
  float speed_mps() const { return speed_; }

 private:
  int32_t ticks_[kSlots] = {};
  uint32_t t_ms_[kSlots] = {};
  int head_ = 0;
  int count_ = 0;
  float speed_ = 0.0f;
};

}  // namespace dbw
