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

#include "actuators.h"

#include <math.h>

#include "dbw_limits.h"

namespace dbw {

namespace {

float Clamp(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }

}  // namespace

void SteeringLoop::Reset() {
  integ_ = 0.0f;
  stall_timing_ = false;
  backing_off_ = false;
}

float SteeringLoop::Step(float setpoint_rad, float angle_rad, float rate_rad_s, float dt_s,
                         uint32_t now_ms, const Params &p, uint16_t *faults) {
  const float sp = Clamp(setpoint_rad, p.steer_limit_rad);
  float err = sp - angle_rad;
  const bool in_band = fabsf(err) < p.steer_deadband_rad;
  if (in_band) err = 0.0f;

  // PID. D acts on the measured rate, not the error, so a setpoint step does
  // not kick the motor.
  const float pterm = p.steer_kp * err;
  const float dterm = -p.steer_kd * rate_rad_s;
  float integ = Clamp(integ_ + p.steer_ki * err * dt_s, p.steer_i_max);
  float u = pterm + integ + dterm;
  const float u_sat = Clamp(u, p.steer_effort_max);
  // Anti-windup: keep the new integral only if it is not pushing further
  // into a saturation the output already sits in.
  if (u != u_sat && (u - u_sat) * err > 0.0f) integ = integ_;
  integ_ = integ;
  u = u_sat;

  // Mechanical-limit clamp: past the stop, effort towards centre only.
  // safety.md row 7 / docs/build/04-firmware.md 4.4.
  if (angle_rad >= p.steer_stop_rad) {
    *faults |= fault::kAtLimit;
    if (u > 0.0f) u = 0.0f;
    if (integ_ > 0.0f) integ_ = 0.0f;
  } else if (angle_rad <= -p.steer_stop_rad) {
    *faults |= fault::kAtLimit;
    if (u < 0.0f) u = 0.0f;
    if (integ_ < 0.0f) integ_ = 0.0f;
  }

  // Stall: effort, but the angle stays inside a small band, for long enough.
  // There is no motor encoder on the stock steering gearmotor, so this is
  // judged on the absolute sensor (03-electrical.md 3.5, encoder row).
  if (backing_off_) {
    if ((int32_t)(now_ms - backoff_until_ms_) >= 0) {
      backing_off_ = false;
      stall_timing_ = false;
    } else {
      *faults |= fault::kStall;
      u = Clamp(u, limits::kStallBackoffEffort);
      if (u * angle_rad > 0.0f) u = 0.0f;  // towards centre only
      integ_ = 0.0f;
      return u;
    }
  }

  if (fabsf(u) >= limits::kStallEffort) {
    if (!stall_timing_) {
      stall_timing_ = true;
      stall_since_ms_ = now_ms;
      stall_angle_ = angle_rad;
    } else if (fabsf(angle_rad - stall_angle_) > limits::kStallBandRad) {
      // It moved: not a stall. Restart the clock from here.
      stall_since_ms_ = now_ms;
      stall_angle_ = angle_rad;
    } else if ((uint32_t)(now_ms - stall_since_ms_) >= limits::kStallMs) {
      backing_off_ = true;
      backoff_until_ms_ = now_ms + limits::kStallBackoffMs;
      *faults |= fault::kStall;
      integ_ = 0.0f;
      u = Clamp(u, limits::kStallBackoffEffort);
      if (u * angle_rad > 0.0f) u = 0.0f;
    }
  } else {
    stall_timing_ = false;
  }
  return u;
}

float ThrottleShaper::Step(float speed_cmd_mps, float wheel_speed_mps, float dt_s,
                           const Params &p) {
  float s = Clamp(speed_cmd_mps, p.speed_cap_mps);

  // Direction interlock: no reversal above a threshold speed. Coast to below
  // it first; the command is honoured once the vehicle has slowed.
  if (s * wheel_speed_mps < 0.0f && fabsf(wheel_speed_mps) > p.reversal_speed_mps) s = 0.0f;

  const float target = Clamp(s * p.drive_effort_per_mps, p.drive_effort_max);

  // Ramp. Growing in magnitude, same direction: accel rate. Anything else -
  // shrinking, or heading for the other direction - decel rate, and a change
  // of direction stops at zero for one tick before it grows again.
  const bool growing = fabsf(target) > fabsf(effort_) && target * effort_ >= 0.0f;
  const float max_step = (growing ? p.accel_effort_per_s : p.decel_effort_per_s) * dt_s;
  float next = effort_ + Clamp(target - effort_, max_step);
  if (effort_ != 0.0f && next * effort_ < 0.0f) next = 0.0f;
  effort_ = next;
  return effort_;
}

}  // namespace dbw
