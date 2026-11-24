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
// Effort -> servo pulse width, for the MDDS30 in R/C mode: 1000..2000 us,
// 1500 us = stop.
//
// TWO WIRING RULES, because Layer B passes the transmitter's CH1/CH2 to the
// driver untouched and the Teensy must mean the same thing by a pulse:
//
//   RC1: a pulse ABOVE 1500 us turns the road wheels RIGHT.
//   RC2: a pulse ABOVE 1500 us drives FORWARD.
//
// Check both at Stage 1 with the Teensy emitting a small fixed effort. If
// either is backwards, SWAP THE MOTOR LEADS AT THE DRIVER'S OUTPUT - not a
// sign in this file. A sign here would make Layer A right and leave Layer B,
// the path that works with this firmware dead, backwards. (Same rule as
// docs/build/07-manual-drive.md: "Fix the wire. Do not flip the sign.")
// If the TRANSMITTER's stick is backwards, reverse that channel on the
// transmitter; that fixes both layers at once.

#pragma once

namespace dbw {

constexpr float kPulseNeutralUs = 1500.0f;
constexpr float kPulseHalfSpanUs = 500.0f;

inline float Clamp1(float x) { return x > 1.0f ? 1.0f : (x < -1.0f ? -1.0f : x); }

// + effort = towards + angle = LEFT, which is a pulse BELOW 1500.
inline float SteerPulseUs(float effort) {
  return kPulseNeutralUs - Clamp1(effort) * kPulseHalfSpanUs;
}

// + effort = FORWARD, a pulse ABOVE 1500.
inline float DrivePulseUs(float effort) {
  return kPulseNeutralUs + Clamp1(effort) * kPulseHalfSpanUs;
}

}  // namespace dbw
