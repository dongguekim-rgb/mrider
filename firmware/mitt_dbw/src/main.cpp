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
// MRider DBW controller - Teensy 4.1.
//
// Two threads of control, and only one of them can move the vehicle:
//
//   control tick (interrupt, 250 Hz)  angle, encoder, i-BUS, E-stop sense,
//                                      state machine, both pulses, relay
//                                      gate, watchdog            control.cpp
//   main loop                          micro-ROS, IMU            ros_link.cpp
//                                                                imu.cpp
//
// The order in setup() is the safety argument in miniature, so it is not to
// be rearranged casually.

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "hw.h"
#include "imu.h"
#include "persist.h"
#include "ros_link.h"

void setup() {
  // 1. The watchdog, before any output is driven (safety.md 2, row 8 note).
  hw::WatchdogBegin(cfg::kWatchdogMs);

  // 2. Both pulse lines to exactly 1500 us, the relay gate low (STOCK).
  //    Until this line the pins are inputs: no pulses, and the driver's own
  //    signal-loss timeout holds the motors.
  hw::PulsesBegin();

  // 3. Calibration from EEPROM, or the defaults - which cannot arm.
  dbw::Params params;
  const bool loaded = persist::Load(&params);

  // 4. The control tick. From here it feeds the watchdog.
  control::Begin(params);

  if (hw::ResetWasWatchdog()) control::PostEvent("boot: after a WATCHDOG reset");
  control::PostEvent(loaded ? "boot: params loaded from EEPROM"
                            : "boot: no saved params - defaults, steering NOT zeroed");

  // 5. The slow things. The tick is already running underneath them.
  ros_link::Begin();
  imu::Begin();
}

void loop() {
  control::Heartbeat();

  imu::Sample s;
  if (imu::Poll(&s)) ros_link::OnImu(s);

  ros_link::Spin();
}
