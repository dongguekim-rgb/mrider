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

#include "imu.h"

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace imu {

namespace {

// The breakout's address is open as of 2026-11-16 (sensors.md 3): 0x4A or
// 0x4B depending on its ADR/DI pin. Both are tried.
constexpr uint8_t kAddrs[] = {0x4A, 0x4B};

Adafruit_BNO08x g_bno(-1);  // no reset pin wired
bool g_up = false;
uint8_t g_addr = 0;
uint32_t g_last_sample_ms = 0;
uint32_t g_last_try_ms = 0;
bool g_tried = false;

Sample g_work;
bool g_have_gyro = false;
bool g_have_acc = false;
char g_desc[64] = "imu: not started";

bool EnableReports() {
  return g_bno.enableReport(SH2_GAME_ROTATION_VECTOR, cfg::kImuReportUs) &&
         g_bno.enableReport(SH2_GYROSCOPE_CALIBRATED, cfg::kImuReportUs) &&
         g_bno.enableReport(SH2_ACCELEROMETER, cfg::kImuReportUs);
}

void TryStart() {
  g_tried = true;
  g_last_try_ms = millis();
  g_up = false;
  for (uint8_t a : kAddrs) {
    // begin_I2C resets the part and waits ~300 ms. That is the main loop
    // waiting, not the steering.
    if (g_bno.begin_I2C(a, &Wire1) && EnableReports()) {
      g_up = true;
      g_addr = a;
      g_last_sample_ms = millis();
      snprintf(g_desc, sizeof(g_desc), "imu: started at 0x%02X", a);
      return;
    }
  }
  snprintf(g_desc, sizeof(g_desc), "imu: no BNO085 at 0x4A or 0x4B on Wire1");
}

}  // namespace

void Begin() {
  Wire1.begin();
  Wire1.setClock(cfg::kImuI2cHz);
  TryStart();
}

bool Poll(Sample *out) {
  const uint32_t now = millis();
  if (!g_up) {
    if (!g_tried || (uint32_t)(now - g_last_try_ms) >= cfg::kImuRetryMs) TryStart();
    return false;
  }

  if (g_bno.wasReset()) {
    // The part reset itself (brown-out, ESD): its reports are gone.
    if (!EnableReports()) {
      g_up = false;
      snprintf(g_desc, sizeof(g_desc), "imu: reset and would not re-enable");
      return false;
    }
  }

  // Bounded: at most a few reports per pass, so a chatty sensor cannot hold
  // the main loop either.
  bool complete = false;
  sh2_SensorValue_t v;
  for (int n = 0; n < 6 && g_bno.getSensorEvent(&v); ++n) {
    g_last_sample_ms = now;
    switch (v.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        g_work.qx = v.un.gameRotationVector.i;
        g_work.qy = v.un.gameRotationVector.j;
        g_work.qz = v.un.gameRotationVector.k;
        g_work.qw = v.un.gameRotationVector.real;
        g_work.quat_status = v.status & 0x03;
        g_work.t_us = micros();
        if (g_have_gyro && g_have_acc) {
          *out = g_work;
          complete = true;
        }
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        g_work.gx = v.un.gyroscope.x;
        g_work.gy = v.un.gyroscope.y;
        g_work.gz = v.un.gyroscope.z;
        g_work.gyro_status = v.status & 0x03;
        g_have_gyro = true;
        break;
      case SH2_ACCELEROMETER:
        g_work.ax = v.un.accelerometer.x;
        g_work.ay = v.un.accelerometer.y;
        g_work.az = v.un.accelerometer.z;
        g_work.acc_status = v.status & 0x03;
        g_have_acc = true;
        break;
      default:
        break;
    }
  }

  if ((uint32_t)(now - g_last_sample_ms) > cfg::kImuStaleMs) {
    // Pulled lead, dead part. Stop, and start again later.
    if ((uint32_t)(now - g_last_sample_ms) > cfg::kImuRetryMs) {
      g_up = false;
      g_have_gyro = g_have_acc = false;
      snprintf(g_desc, sizeof(g_desc), "imu: silent for %lu ms, restarting",
               (unsigned long)(now - g_last_sample_ms));
    }
    return false;
  }
  if (complete) {
    snprintf(g_desc, sizeof(g_desc), "imu: ok 0x%02X, accuracy q%u g%u a%u", g_addr,
             g_work.quat_status, g_work.gyro_status, g_work.acc_status);
  }
  return complete;
}

bool Healthy() { return g_up && (uint32_t)(millis() - g_last_sample_ms) <= cfg::kImuStaleMs; }

const char *Describe() { return g_desc; }

}  // namespace imu
