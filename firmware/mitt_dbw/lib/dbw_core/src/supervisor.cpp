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

#include "supervisor.h"

#include <math.h>

#include "dbw_limits.h"

namespace dbw {

namespace {

float Clamp(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }

}  // namespace

void Supervisor::EnterEstop(const Inputs &in, const char *why, Outputs *out) {
  // Coming from an armed mode, the coils are energised and the driver has
  // power: centre the steering before letting go (safety.md rows 1 and 3).
  centring_ = (mode_ != Mode::kEstop);
  mode_ = Mode::kEstop;
  estop_entered_ms_ = in.now_ms;
  centred_timing_ = false;
  throttle_.Reset();  // throttle -> 0 now, not down a ramp
  steer_.Reset();
  out->event = why;
}

void Supervisor::Arm(const Inputs &in, Mode target, Outputs *out) {
  mode_ = target;
  armed_at_ms_ = in.now_ms;
  centring_ = false;
  steer_.Reset();
  throttle_.Reset();
  out->event = (target == Mode::kAutonomous) ? "armed: AUTONOMOUS" : "armed: MANUAL_RC";
}

Outputs Supervisor::Step(const Inputs &in, const Params &p) {
  Outputs out;

  if (!started_) {
    // Power-up counts as an E-stop release: the driver may be booting too.
    started_ = true;
    estop_clear_since_ms_ = in.now_ms;
  }

  // ---- facts this tick ----------------------------------------------------
  const bool cmd_ok = in.cmd_age_ms <= limits::kCmdStaleMs;
  const bool rc_ok = in.rc_age_ms <= limits::kRcTimeoutMs;
  if (!cmd_ok) out.faults |= fault::kSetpointStale;
  if (!rc_ok) out.faults |= fault::kRcLoss;
  if (!in.angle_ok) out.faults |= fault::kEncoder;

  if (!in.estop_high) estop_clear_since_ms_ = in.now_ms;
  const bool in_holdoff = (uint32_t)(in.now_ms - estop_clear_since_ms_) < limits::kEstopHoldoffMs;
  const bool estop_blocked = !in.estop_high || in_holdoff;
  if (estop_blocked) out.faults |= fault::kEstop;

  const RcSwitch sw = rc_ok ? in.rc_switch : RcSwitch::kUnknown;
  const bool left_kill = prev_sw_ == RcSwitch::kKill &&
                         (sw == RcSwitch::kManual || sw == RcSwitch::kAuto);
  const bool to_auto = prev_sw_ == RcSwitch::kManual && sw == RcSwitch::kAuto;
  prev_sw_ = sw;

  const bool sticks_centred = fabsf(in.rc_steer) < limits::kStickCentred &&
                              fabsf(in.rc_throttle) < limits::kStickCentred;
  const bool sticks_taken = fabsf(in.rc_steer) > limits::kStickTakeover ||
                            fabsf(in.rc_throttle) > limits::kStickTakeover;

  // ---- transitions ----------------------------------------------------------
  if (mode_ == Mode::kEstop) {
    if (left_kill) {
      const Mode target = (sw == RcSwitch::kAuto) ? Mode::kAutonomous : Mode::kManualRc;
      const char *refuse = nullptr;
      if (!in.estop_high) {
        refuse = "arm refused: E-stop sense is low";
      } else if (in_holdoff) {
        refuse = "arm refused: inside the E-stop hold-off";
      } else if (!in.steer_zeroed) {
        refuse = "arm refused: steering never zeroed (/mitt_dbw/zero_steering)";
      } else if (!in.angle_ok) {
        refuse = "arm refused: steering angle not trusted";
      } else if (!sticks_centred) {
        refuse = "arm refused: sticks not centred";
      } else if (target == Mode::kAutonomous && !cmd_ok) {
        refuse = "arm refused: AUTO needs a live /mitt/dbw/command";
      }
      if (refuse) {
        out.event = refuse;
      } else {
        Arm(in, target, &out);
      }
    }
  } else {
    const char *why = nullptr;
    if (!in.estop_high) {
      why = "ESTOP: E-stop sense low";
    } else if (!rc_ok) {
      why = "ESTOP: RC link lost";
    } else if (sw == RcSwitch::kKill) {
      why = "ESTOP: RC kill switch";
    } else if (!in.angle_ok) {
      why = "ESTOP: steering angle sensor fault";
    } else if (mode_ == Mode::kAutonomous && !cmd_ok) {
      why = "ESTOP: /mitt/dbw/command stale > 500 ms";
    } else if (to_auto && !cmd_ok) {
      why = "ESTOP: AUTO selected with no live /mitt/dbw/command";
    }

    if (why) {
      EnterEstop(in, why, &out);
    } else if (mode_ == Mode::kManualRc && to_auto) {
      mode_ = Mode::kAutonomous;
      out.event = "mode: AUTONOMOUS";
    } else if (mode_ == Mode::kAutonomous && sw == RcSwitch::kManual) {
      mode_ = Mode::kManualRc;
      out.event = "mode: MANUAL_RC (switch)";
    } else if (mode_ == Mode::kAutonomous && sticks_taken) {
      mode_ = Mode::kManualRc;
      out.event = "mode: MANUAL_RC (sticks taken)";
    }
  }
  out.mode = mode_;

  // ---- outputs ------------------------------------------------------------
  // E-stop sense low, or the hold-off after it: nothing but exact neutral on
  // both lines, and no loop state carried across. 04-firmware.md 4.3,
  // behaviours 2 and 3 - the driver calibrates its centre on what it sees
  // when it powers up.
  if (estop_blocked) {
    steer_.Reset();
    throttle_.Reset();
    centring_ = false;
    out.relay_dbw = false;
    return out;
  }

  if (mode_ == Mode::kEstop) {
    out.drive_effort = 0.0f;
    const bool timed_out =
        (uint32_t)(in.now_ms - estop_entered_ms_) >= limits::kCentreTimeoutMs;
    if (centring_ && in.angle_ok && !timed_out) {
      uint16_t f = 0;
      out.setpoint_rad = 0.0f;
      out.steer_effort = steer_.Step(0.0f, in.angle_rad, in.angle_rate_rad_s, in.dt_s,
                                     in.now_ms, p, &f);
      out.faults |= f;
      out.relay_dbw = true;
      if (fabsf(in.angle_rad) < limits::kCentreTolRad) {
        if (!centred_timing_) {
          centred_timing_ = true;
          centred_since_ms_ = in.now_ms;
        } else if ((uint32_t)(in.now_ms - centred_since_ms_) >= limits::kCentreHoldMs) {
          centring_ = false;
          out.event = "ESTOP: steering centred, released to STOCK";
        }
      } else {
        centred_timing_ = false;
      }
    } else {
      if (centring_) out.event = "ESTOP: centring abandoned, released to STOCK";
      centring_ = false;
      steer_.Reset();
      out.steer_effort = 0.0f;
      out.relay_dbw = false;
    }
    return out;
  }

  // Armed.
  out.relay_dbw = true;
  float setpoint;
  float speed;
  if (mode_ == Mode::kManualRc) {
    setpoint = in.rc_steer * p.steer_limit_rad;
    speed = in.rc_throttle * p.rc_speed_max_mps;
  } else {
    setpoint = in.cmd_steer_rad;
    speed = in.cmd_speed_mps;
  }
  out.setpoint_rad = Clamp(setpoint, p.steer_limit_rad);

  if ((uint32_t)(in.now_ms - armed_at_ms_) < limits::kArmSettleMs) {
    // The relay contacts are still settling: switch no current through them.
    return out;
  }

  uint16_t f = 0;
  out.steer_effort = steer_.Step(out.setpoint_rad, in.angle_rad, in.angle_rate_rad_s, in.dt_s,
                                 in.now_ms, p, &f);
  out.faults |= f;
  out.drive_effort = throttle_.Step(speed, in.wheel_speed_mps, in.dt_s, p);
  return out;
}

}  // namespace dbw
