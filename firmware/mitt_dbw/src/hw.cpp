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

#include "hw.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace hw {

// ============================================================ AS5600
//
// Same registers and the same "is the magnet there" rule as the bench sketch
// firmware/bench/src/as5600_check.cpp, which proved them first.

namespace {

constexpr uint8_t kAs5600 = 0x36;
constexpr uint8_t kRegStatus = 0x0B;
constexpr uint8_t kRegRawHi = 0x0C;
constexpr uint8_t kStatusMD = 0x20;  // magnet detected
constexpr uint8_t kStatusML = 0x10;  // too weak
constexpr uint8_t kStatusMH = 0x08;  // too strong

bool ReadBytes(uint8_t reg, uint8_t *out, uint8_t n) {
  Wire.beginTransmission(kAs5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kAs5600, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) out[i] = (uint8_t)Wire.read();
  return true;
}

}  // namespace

void AngleBegin() {
  Wire.begin();
  Wire.setClock(cfg::kAngleI2cHz);
}

dbw::AngleSensor::Sample AngleRead() {
  dbw::AngleSensor::Sample s;
  uint8_t status = 0;
  uint8_t raw[2] = {0, 0};
  if (!ReadBytes(kRegStatus, &status, 1) || !ReadBytes(kRegRawHi, raw, 2)) return s;
  s.read_ok = true;
  s.magnet_ok = (status & kStatusMD) && !(status & kStatusML) && !(status & kStatusMH);
  s.raw = (uint16_t)(((raw[0] & 0x0F) << 8) | raw[1]);
  return s;
}

// ============================================================ pulses

namespace {

constexpr int kPwmBits = 16;
constexpr float kPwmFull = 65535.0f;

uint32_t UsToDuty(float us) {
  const float period_us = 1.0e6f / (float)cfg::kPwmHz;
  float duty = us / period_us * kPwmFull;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > kPwmFull) duty = kPwmFull;
  return (uint32_t)(duty + 0.5f);
}

}  // namespace

void PulsesBegin() {
  // Pins 2 and 3 are one FlexPWM submodule (4.2 A/B), so one frequency.
  analogWriteResolution(kPwmBits);
  analogWriteFrequency(cfg::kPinPwmSteer, cfg::kPwmHz);
  analogWriteFrequency(cfg::kPinPwmDrive, cfg::kPwmHz);
  PulsesWrite(1500.0f, 1500.0f);
  RelayGate(false);
}

void PulsesWrite(float steer_us, float drive_us) {
  analogWrite(cfg::kPinPwmSteer, UsToDuty(steer_us));
  analogWrite(cfg::kPinPwmDrive, UsToDuty(drive_us));
}

void RelayGate(bool dbw) {
  // The 10 k pulldown on the relay board holds the gate low - STOCK - until
  // this pin is driven, and again whenever the Teensy is reset or unplugged.
  static bool configured = false;
  if (!configured) {
    digitalWriteFast(cfg::kPinRelayGate, LOW);
    pinMode(cfg::kPinRelayGate, OUTPUT);
    configured = true;
  }
  digitalWriteFast(cfg::kPinRelayGate, dbw ? HIGH : LOW);
}

// ============================================================ watchdog
//
// RTWDOG = WDOG3 on the i.MX RT1062, clocked from the 32 kHz LPO. A timeout
// resets the whole chip, and CrashReport / SRC_SRSR says it was WDOG3.
//
// The 32 kHz figure is the clock this code assumes; the Stage 1 hang test
// measures the real period. If the motor stops ~30x later than kWatchdogMs,
// the clock is the 1 kHz option and this is the line to fix.

namespace {

constexpr uint32_t kUnlockKey = 0xD928C520;
constexpr uint32_t kRefreshKey = 0xB480A602;
constexpr uint32_t kWdogClockHz = 32000;
bool g_reset_was_wdog = false;

void WatchdogConfigure(uint32_t timeout_ms) {
  uint32_t ticks = timeout_ms * (kWdogClockHz / 1000);
  if (ticks > 0xFFFF) ticks = 0xFFFF;  // ~2 s without the prescaler
  if (ticks < 16) ticks = 16;

  __disable_irq();
  WDOG3_CNT = kUnlockKey;
  __asm__ volatile("dsb");
  while (!(WDOG3_CS & WDOG_CS_ULK)) {
  }
  WDOG3_WIN = 0;
  WDOG3_TOVAL = ticks;
  WDOG3_CS = WDOG_CS_EN | WDOG_CS_CLK(1) | WDOG_CS_UPDATE | WDOG_CS_CMD32EN | WDOG_CS_WAIT;
  while (!(WDOG3_CS & WDOG_CS_RCS)) {
  }
  __enable_irq();
}

}  // namespace

void WatchdogBegin(uint32_t timeout_ms) {
  g_reset_was_wdog = (SRC_SRSR & SRC_SRSR_WDOG3_RST_B) != 0;
  CCM_CCGR5 |= CCM_CCGR5_WDOG3(CCM_CCGR_ON);
  WatchdogConfigure(timeout_ms);
}

void WatchdogSetTimeout(uint32_t timeout_ms) {
  WatchdogFeed();
  WatchdogConfigure(timeout_ms);
  WatchdogFeed();
}

void WatchdogFeed() { WDOG3_CNT = kRefreshKey; }

bool ResetWasWatchdog() { return g_reset_was_wdog; }

}  // namespace hw
