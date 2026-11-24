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

#include "sensors.h"

#include <math.h>

#include "dbw_limits.h"

namespace dbw {

namespace {
// The rate is filtered for the D term. 20 ms keeps one count of quantisation
// from reading as a lurch.
constexpr float kRateTauS = 0.020f;
}  // namespace

int32_t AngleSensor::Wrap(int32_t counts) {
  counts %= 4096;
  if (counts < -2048) counts += 4096;
  if (counts >= 2048) counts -= 4096;
  return counts;
}

void AngleSensor::Update(const Sample &s, float dt_s, const Params &p) {
  if (!s.read_ok || !s.magnet_ok) {
    ok_ = false;
    have_last_ = false;  // continuity is broken; do not bridge it
    good_streak_ = 0;
    rate_ = 0.0f;
    return;
  }

  raw_ = s.raw;
  const int32_t c0 = p.steer_zero_counts >= 0 ? p.steer_zero_counts : 0;
  const float angle = (float)Wrap((int32_t)s.raw - c0) * p.steer_rad_per_count;

  bool plausible = true;
  // Range is only judged once zeroed. Before that the angle is relative to
  // an arbitrary 0 and means nothing - and arming is refused anyway.
  if (p.steer_zero_counts >= 0 &&
      fabsf(angle) > p.steer_stop_rad + limits::kAnglePastStopRad) {
    plausible = false;
  }
  if (have_last_ &&
      fabsf(angle - last_angle_) > limits::kMaxRateRadS * dt_s + 4.0f * fabsf(p.steer_rad_per_count)) {
    plausible = false;
  }

  if (have_last_ && dt_s > 0.0f) {
    const float raw_rate = (angle - last_angle_) / dt_s;
    const float a = dt_s / (kRateTauS + dt_s);
    rate_ += a * (raw_rate - rate_);
  }
  last_angle_ = angle;
  have_last_ = true;
  angle_ = angle;

  if (!plausible) {
    ok_ = false;
    good_streak_ = 0;
    return;
  }
  if (good_streak_ < limits::kAngleRecoverSamples) ++good_streak_;
  ok_ = good_streak_ >= limits::kAngleRecoverSamples;
}

void WheelSpeed::Update(int32_t ticks, uint32_t now_ms, float m_per_tick) {
  head_ = (head_ + 1) % kSlots;
  ticks_[head_] = ticks;
  t_ms_[head_] = now_ms;
  if (count_ < kSlots) ++count_;

  // Oldest slot still inside the window, but at least one step back.
  int oldest = head_;
  for (int n = 1; n < count_; ++n) {
    const int i = (head_ - n + kSlots) % kSlots;
    oldest = i;
    if ((uint32_t)(now_ms - t_ms_[i]) >= kWindowMs) break;
  }
  const uint32_t span_ms = (uint32_t)(now_ms - t_ms_[oldest]);
  if (oldest == head_ || span_ms == 0) {
    speed_ = 0.0f;
    return;
  }
  speed_ = (float)(ticks - ticks_[oldest]) * m_per_tick * 1000.0f / (float)span_ms;
}

}  // namespace dbw
