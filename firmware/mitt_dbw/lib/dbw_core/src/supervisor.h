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
// The safety state machine of docs/design/dbw.md 10.3, and the output rules
// that go with each state. One Step() per control tick.
//
//          ESTOP  <-- E-stop low / RC kill / RC loss / angle fault /
//            |        laptop stale (AUTO only)
//            | ARM: CH6 moved out of KILL, with
//            |      E-stop high and its hold-off over, steering zeroed,
//            |      angle trusted, RC live, sticks centred,
//            |      and for AUTO a live laptop command
//            v
//      MANUAL_RC  <-->  AUTONOMOUS
//         CH6 -> AUTO (edge)     CH6 -> MANUAL, or the sticks taken
//
// WHAT "ARM" MEANS, AND WHY IT IS AN EDGE. Releasing the E-stop, the RC link
// coming back, the laptop starting to publish: none of these is a "go". The
// only way out of ESTOP is the operator moving CH6 out of KILL while every
// condition holds at that moment. A request that fails is dropped, not
// queued - try again from KILL. (04-firmware.md 4.3: "An arm request during
// the hold-off is ignored, not queued.")
//
// THE RELAY MUX. Pin 21 energises the relay MUX coils, which hand the motors
// from the stock controller to the MDDS30. It is high while armed, and while
// ESTOP is still centring the steering; otherwise low. So ESTOP ends in
// STOCK, as safety.md 1.1 has every abort do - and the coils only ever
// change over with both pulse lines at neutral.

#pragma once

#include <stdint.h>

#include "actuators.h"
#include "dbw_types.h"

namespace dbw {

class Supervisor {
 public:
  Outputs Step(const Inputs &in, const Params &p);

  Mode mode() const { return mode_; }

 private:
  void EnterEstop(const Inputs &in, const char *why, Outputs *out);
  void Arm(const Inputs &in, Mode target, Outputs *out);

  bool started_ = false;
  Mode mode_ = Mode::kEstop;
  RcSwitch prev_sw_ = RcSwitch::kUnknown;

  uint32_t estop_clear_since_ms_ = 0;  // when the sense was last seen low

  uint32_t armed_at_ms_ = 0;
  bool centring_ = false;
  uint32_t estop_entered_ms_ = 0;
  bool centred_timing_ = false;
  uint32_t centred_since_ms_ = 0;

  SteeringLoop steer_;
  ThrottleShaper throttle_;
};

}  // namespace dbw
