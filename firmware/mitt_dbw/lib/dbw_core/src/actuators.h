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

// The steering position loop (dbw.md 10.2 block 3) with the two interlocks
// that live next to it (block 7): the mechanical-limit clamp and stall
// back-off. Output is effort, -1..1, + = towards + angle.
class SteeringLoop {
 public:
  void Reset();

  // faults gets kAtLimit / kStall OR-ed in when they apply this tick.
  float Step(float setpoint_rad, float angle_rad, float rate_rad_s, float dt_s,
             uint32_t now_ms, const Params &p, uint16_t *faults);

  float integrator() const { return integ_; }

 private:
  float integ_ = 0.0f;
  bool stall_timing_ = false;
  uint32_t stall_since_ms_ = 0;
  float stall_angle_ = 0.0f;
  bool backing_off_ = false;
  uint32_t backoff_until_ms_ = 0;
};

// Throttle shaping (dbw.md 7, block 5): speed cap, direction interlock, ramp.
class ThrottleShaper {
 public:
  void Reset() { effort_ = 0.0f; }
  float Step(float speed_cmd_mps, float wheel_speed_mps, float dt_s, const Params &p);
  float effort() const { return effort_; }

 private:
  float effort_ = 0.0f;
};

}  // namespace dbw
