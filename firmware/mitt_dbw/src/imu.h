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
//
// BNO085 on Wire1 (SDA1 = 17, SCL1 = 16) - docs/design/sensors.md 3.
//
// Main loop only. Nothing here can delay the steering: the steering runs in
// the control tick, on the other I2C bus.
//
// Orientation is the GAME rotation vector - accelerometer and gyro, no
// magnetometer - because ros2_ws/src/mitt_localization/config/ekf_indoor.yaml
// fuses the IMU with imu0_relative: true, "no magnetometer". Indoors, beside
// two motors, a magnetometer heading is a liability.

#pragma once

#include <stdint.h>

namespace imu {

struct Sample {
  uint32_t t_us = 0;  // micros() when the orientation report was read
  float qx = 0, qy = 0, qz = 0, qw = 1;
  float gx = 0, gy = 0, gz = 0;  // rad/s
  float ax = 0, ay = 0, az = 0;  // m/s^2, gravity included (REP 145)
  uint8_t acc_status = 0;        // 0..3 per report: 0 unreliable, 3 high
  uint8_t gyro_status = 0;
  uint8_t quat_status = 0;
};

void Begin();

// Call every main-loop pass. True when a new, complete sample is in *out.
bool Poll(Sample *out);

// False when no sample has arrived for cfg::kImuStaleMs. Not publishing is
// then the right thing (04-firmware.md: silence, not a stale sample).
bool Healthy();

// Short status for the event topic, e.g. "imu: ok 0x4A, accuracy q3 g3 a2".
const char *Describe();

}  // namespace imu
