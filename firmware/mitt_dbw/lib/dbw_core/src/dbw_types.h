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
// The types every part of the DBW core shares. No Arduino here: lib/dbw_core
// holds the decisions, src/ holds the hardware.
//
// SIGN CONVENTIONS - one place, used everywhere
// ---------------------------------------------
//   steering angle   + = LEFT  (ROS REP 103: positive yaw is counter-clockwise)
//   steering effort  + = drives the angle towards +, i.e. LEFT
//   drive effort     + = FORWARD
//   wheel speed      + = FORWARD
//
// What reaches the wire is fixed by the R/C MUX, not chosen here. Through
// Layer B the transmitter's CH1 and CH2 go to the driver untouched, so the
// Teensy has to mean the same thing by a pulse width that the transmitter
// does - see pulse.h for the two rules that follow.

#pragma once

#include <stdint.h>

namespace dbw {

// Values are DbwStatus.MODE_* - they go on the wire as they are.
enum class Mode : uint8_t {
  kEstop = 0,
  kManualRc = 1,
  kAutonomous = 2,
};

// DbwStatus.faults. 1..32 are the message's own constants; 64 is the E-stop
// sense, which docs/build/04-firmware.md 4.3 requires in the status and the
// message had no field for.
namespace fault {
constexpr uint16_t kEncoder = 1;         // angle sensor NAK / out of range / magnet lost
constexpr uint16_t kSetpointStale = 2;   // no DbwCommand within 500 ms
constexpr uint16_t kAtLimit = 4;         // steering at or past the mechanical stop
constexpr uint16_t kStall = 8;           // effort with no motion - backing off
constexpr uint16_t kRcLoss = 16;         // no valid i-BUS frame within the timeout
constexpr uint16_t kOverCurrent = 32;    // NEVER SET: the MDDS30 reports nothing in R/C mode
constexpr uint16_t kEstop = 64;          // E-stop sense low, or inside the hold-off after it
}  // namespace fault

// The transmitter's 3-position mode switch (CH6). kUnknown = no RC link.
enum class RcSwitch : uint8_t {
  kUnknown = 0,
  kKill = 1,
  kManual = 2,
  kAuto = 3,
};

// Everything the vehicle can be tuned or calibrated with. Persisted to EEPROM
// (src/persist.cpp) and exposed as ROS parameters (src/ros_link.cpp).
//
// EVERY DEFAULT BELOW THAT DESCRIBES THE VEHICLE IS A PLACEHOLDER. They are
// chosen to be harmless on a current-limited bench supply, not to be right.
// The procedure that replaces each one is named beside it.
struct Params {
  // --- steering calibration: docs/design/calibration.md 1 -----------------
  // c0: raw AS5600 counts with the wheels straight. -1 = never zeroed, and
  // the supervisor refuses to arm until /mitt_dbw/zero_steering is called.
  int32_t steer_zero_counts = -1;
  // k, signed. 2*pi/4096 assumes the sensor turns 1:1 with the road wheels,
  // which it does not - the linkage sits between. calibration.md 1.2.
  float steer_rad_per_count = 0.0015339808f;
  // Commandable range: dbw.md 12, +/-22.5 deg. Commands are clamped to it.
  float steer_limit_rad = 0.3927f;
  // Where the mechanical stop is. Past it, effort is allowed towards centre
  // only and FAULT_AT_LIMIT is set. Measure lock-to-lock and set it.
  float steer_stop_rad = 0.4200f;

  // --- steering position loop: docs/build/04-firmware.md 4.4 -------------
  // Start P only, add D, add I last and sparingly. Units: effort (-1..1) per
  // radian of error, per radian-second, per radian/second.
  float steer_kp = 3.0f;
  float steer_ki = 0.0f;
  float steer_kd = 0.0f;
  float steer_i_max = 0.3f;          // the integrator's share of effort, at most
  float steer_deadband_rad = 0.0087f;  // 0.5 deg: dbw.md 2.2.2, rung R1
  float steer_effort_max = 0.6f;     // fraction of full R/C deflection

  // --- drive: docs/design/dbw.md 7 and calibration.md 2 -------------------
  // Roll-out calibration replaces this: 10 m / measured ticks.
  // Placeholder = 2*pi*0.09 m wheel / 36 counts per wheel turn.
  float drive_m_per_tick = 0.0157f;
  // Open loop: effort = speed * this. There is no speed loop in the design.
  float drive_effort_per_mps = 0.5f;
  float drive_effort_max = 0.5f;     // dbw.md 7: "a firmware duty ceiling"
  float speed_cap_mps = 1.0f;        // the laptop is never obeyed above this
  float rc_speed_max_mps = 0.8f;     // full stick in MANUAL_RC
  float accel_effort_per_s = 0.8f;   // ramp limit when effort grows
  float decel_effort_per_s = 2.0f;   // and when it shrinks
  float reversal_speed_mps = 0.2f;   // direction interlock threshold
};

// One control tick's view of the world, assembled by the firmware glue.
struct Inputs {
  uint32_t now_ms = 0;
  float dt_s = 0.004f;

  bool estop_high = false;  // pin 20: high = authority chain energised

  // Steering angle, already converted and judged by AngleSensor.
  bool angle_ok = false;
  float angle_rad = 0.0f;
  float angle_rate_rad_s = 0.0f;
  bool steer_zeroed = false;

  // RC, already decoded by DecodeRc. Age of the last valid frame.
  uint32_t rc_age_ms = UINT32_MAX;
  RcSwitch rc_switch = RcSwitch::kUnknown;
  float rc_steer = 0.0f;     // -1..1, + = left
  float rc_throttle = 0.0f;  // -1..1, + = forward

  // Laptop command. Age since the last DbwCommand arrived.
  uint32_t cmd_age_ms = UINT32_MAX;
  float cmd_steer_rad = 0.0f;
  float cmd_speed_mps = 0.0f;

  float wheel_speed_mps = 0.0f;
};

struct Outputs {
  Mode mode = Mode::kEstop;
  uint16_t faults = 0;
  float steer_effort = 0.0f;  // -1..1, + = left. 0 = exactly 1.5 ms
  float drive_effort = 0.0f;  // -1..1, + = forward. 0 = exactly 1.5 ms
  bool relay_dbw = false;     // pin 21: energise the relay MUX (DBW)
  float setpoint_rad = 0.0f;
  // A human-readable line when something happened this tick, else nullptr.
  // Always a string literal - safe to hand from the control ISR to the loop.
  const char *event = nullptr;
};

}  // namespace dbw
