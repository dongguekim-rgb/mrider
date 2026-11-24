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
// The control tick: a timer interrupt at 250 Hz that owns everything the
// vehicle's motion depends on.
//
//   read AS5600 -> read encoder -> drain i-BUS -> read E-stop sense
//   -> Supervisor::Step -> write both pulses and the relay gate
//   -> publish a snapshot -> feed the watchdog
//
// WHY AN INTERRUPT. The main loop does micro-ROS and the IMU, and both can
// block: a USB write waits on the host, a BNO085 re-init sleeps for 300 ms.
// Here none of that can reach the steering. "An IMU read never delays the
// steering loop" (04-firmware.md) is then a property of the structure, not a
// promise each line of the main loop has to keep.
//
// WHY THE WATCHDOG IS FED HERE AND NOT FROM A SEPARATE TIMER. safety.md 2:
// "fed only from the control loop, after an iteration that read the sensor
// and wrote the outputs - a watchdog fed from a timer interrupt survives
// exactly the hang it exists for." This interrupt IS the control loop; the
// feed is its last line, so a hang anywhere in it starves the watchdog.

#pragma once

#include <stdint.h>

#include "dbw_types.h"

namespace control {

struct Snapshot {
  uint32_t meas_us = 0;  // micros() when the angle was read
  float angle_rad = 0.0f;
  float setpoint_rad = 0.0f;
  float wheel_speed_mps = 0.0f;
  int32_t drive_ticks = 0;
  uint8_t mode = 0;
  uint16_t faults = 0;
  uint16_t raw_counts = 0;
  bool raw_ok = false;
  bool relay_dbw = false;
  bool mux_to_receiver = false;
};

struct LoopStats {
  uint32_t ticks = 0;
  uint32_t min_period_us = 0;
  uint32_t max_period_us = 0;
  uint32_t max_exec_us = 0;
  uint32_t overruns = 0;   // periods more than 1.5x nominal
  float hz = 0.0f;
  uint32_t ibus_bad_frames = 0;
};

// Starts the tick. Call with the watchdog already armed.
void Begin(const dbw::Params &p);

// Main loop -> tick.
void SetParams(const dbw::Params &p);
dbw::Params GetParams();
void OnCommand(float steer_rad, float speed_mps);  // a DbwCommand arrived
void Heartbeat();                                  // the main loop came round
void PostEvent(const char *literal);               // from main, for the event topic

// Tick -> main loop.
Snapshot Take();
bool PopEvent(const char **literal);
LoopStats TakeStats();  // and reset them

#ifdef MITT_HANG_TEST
// 04-firmware.md 4.4: hang the control loop on purpose. The next tick
// enters while (true) {} and never feeds the watchdog again.
void RequestHang();
#endif

}  // namespace control
