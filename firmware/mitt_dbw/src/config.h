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
// Pins and rates for the vehicle firmware.
//
// THE PINS ARE COPIED FROM ONE TABLE AND DECIDED NOWHERE ELSE:
// docs/build/04-firmware.md, "The pin assignment". Change a pin there first;
// the bench layout and the soldered signal board read from the same table.

#pragma once

#include <stdint.h>

#if !defined(MITT_VEHICLE) || (MITT_VEHICLE + 0) < 1 || (MITT_VEHICLE + 0) > 3
#error "Build a vehicle environment (pio run -e vehicle2), or for hang_test set MITT_VEHICLE=1..3 in the shell."
#endif

namespace cfg {

// --- pins: docs/build/04-firmware.md --------------------------------------
constexpr uint8_t kPinEncA = 0;         // drive encoder A, QDC channel 1
constexpr uint8_t kPinEncB = 1;         // drive encoder B
constexpr uint8_t kPinPwmSteer = 2;     // -> signal MUX master -> RC1
constexpr uint8_t kPinPwmDrive = 3;     // -> signal MUX master -> RC2
constexpr uint8_t kPinIbusRx = 15;      // Serial3 RX - via R5/R6, never 5 V
constexpr uint8_t kPinImuScl = 16;      // Wire1, fixed by hardware
constexpr uint8_t kPinImuSda = 17;
constexpr uint8_t kPinAngleSda = 18;    // Wire, fixed by hardware
constexpr uint8_t kPinAngleScl = 19;
constexpr uint8_t kPinEstopSense = 20;  // high = authority chain energised
constexpr uint8_t kPinRelayGate = 21;   // high = relay MUX energised = DBW

// --- ROS -------------------------------------------------------------------
// docs/design/software.md 3.4: vehicle N lives in domain 10 + N.
constexpr int kVehicle = MITT_VEHICLE;
constexpr int kRosDomain = 10 + MITT_VEHICLE;

// --- rates -----------------------------------------------------------------
// Servo frame rate to the driver. 50 Hz is the standard frame and the safe
// start. dbw.md 4 leaves the real ceiling OPEN: measure at Stage 1 how fast
// the MDDS30 tracks, then raise this and record the number in dbw.md 12.
constexpr uint32_t kPwmHz = 50;

// The AS5600 is the steering loop's only input. 400 kHz keeps the two reads
// per tick near 0.2 ms; drop to 100000 if the cable to the column is long
// and the bench shows NAKs.
constexpr uint32_t kAngleI2cHz = 400000;
constexpr uint32_t kImuI2cHz = 400000;

// Published rates. dbw.md 12 asks >= 50 Hz status and >= 100 Hz IMU; both
// run above the floor so `ros2 topic hz` does not sit on the line.
constexpr uint32_t kStatusHz = 100;
constexpr uint32_t kImuReportUs = 5000;  // 200 Hz from the sensor

// --- the watchdog: safety.md 2, note under row 8 ---------------------------
// Fed only by a completed control tick. A hang inside the control loop
// stops the feeding, the board resets, the pulses stop, and the driver's own
// 100 ms signal-loss timeout (SW6 = 0) stops the motors. Record this number
// beside the Stage 1 hang-test result.
constexpr uint32_t kWatchdogMs = 100;
// EEPROM writes stop every interrupt while flash is erased. The watchdog is
// stretched to this for the length of a save, and only then.
constexpr uint32_t kWatchdogSaveMs = 2000;

// If the main loop (micro-ROS, IMU) stops coming round for this long, the
// control tick stops feeding the watchdog too, and the board resets. The
// steering never depends on the main loop; this only recovers telemetry.
constexpr uint32_t kMainLoopStallMs = 5000;

// IMU: no new sample for this long -> stop publishing (04-firmware.md: "A
// stale sample republished looks healthy to the EKF; silence does not").
constexpr uint32_t kImuStaleMs = 100;
constexpr uint32_t kImuRetryMs = 5000;

}  // namespace cfg
