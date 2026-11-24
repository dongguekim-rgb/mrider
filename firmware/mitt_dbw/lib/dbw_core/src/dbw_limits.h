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
// The safety numbers. NOT parameters: nothing over the USB link can change
// them, and changing one is a reviewed commit. Each names its source.

#pragma once

#include <stdint.h>

namespace dbw {
namespace limits {

// dbw.md 12 / safety.md row 1: command staleness > 500 ms -> ESTOP.
constexpr uint32_t kCmdStaleMs = 500;

// safety.md row 3. i-BUS sends a frame every ~7.7 ms, so 100 ms is a dozen
// missed frames, and well inside the 200 ms override budget of dbw.md 12.
constexpr uint32_t kRcTimeoutMs = 100;

// docs/build/04-firmware.md 4.3, behaviour 3: after the E-stop sense goes
// high, hold both lines at exactly neutral while the driver boots and
// calibrates. "Start at 1 s and shorten it only against a measurement."
// The same hold-off runs from power-up, when the driver may be booting too.
constexpr uint32_t kEstopHoldoffMs = 1000;

// After an arm, the relay MUX has just been energised. Hold neutral until
// its contacts have stopped bouncing, so no current is switched by them.
constexpr uint32_t kArmSettleMs = 150;

// safety.md rows 1 and 3: "steering centred, then de-energised".
constexpr uint32_t kCentreTimeoutMs = 2000;
constexpr float kCentreTolRad = 0.0175f;  // 1 deg
constexpr uint32_t kCentreHoldMs = 200;

// Sticks must be inside this to arm (safety.md row 5: "transmitter sticks
// centred"), and outside the takeover band to grab the vehicle back from the
// laptop (safety.md 1.2, Layer A: "taking the sticks").
constexpr float kStickCentred = 0.10f;
constexpr float kStickTakeover = 0.30f;

// Stall: safety.md rows 7 and FMEA 6. Effort at least this, the angle moving
// less than the band, for this long -> back off, towards centre only.
constexpr float kStallEffort = 0.40f;
constexpr float kStallBandRad = 0.0087f;  // 0.5 deg
constexpr uint32_t kStallMs = 500;
constexpr uint32_t kStallBackoffMs = 1000;
constexpr float kStallBackoffEffort = 0.15f;

// Angle plausibility, safety.md row 9. A reading further than this past the
// mechanical stop cannot be real. Nor can a jump faster than kMaxRate.
constexpr float kAnglePastStopRad = 0.15f;
constexpr float kMaxRateRadS = 6.0f;
constexpr int kAngleRecoverSamples = 10;

// The control loop's period: dbw.md 12 asks >= 200 Hz.
constexpr uint32_t kLoopHz = 250;

}  // namespace limits
}  // namespace dbw
