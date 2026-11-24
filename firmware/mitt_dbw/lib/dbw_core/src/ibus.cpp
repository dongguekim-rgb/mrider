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

#include "ibus.h"

namespace dbw {

namespace {

constexpr uint8_t kHeader0 = 0x20;
constexpr uint8_t kHeader1 = 0x40;

constexpr uint16_t kUsMin = 900;
constexpr uint16_t kUsMax = 2100;
constexpr float kUsCentre = 1500.0f;
constexpr float kUsHalfSpan = 500.0f;
constexpr float kStickDeadband = 0.03f;

// FS-i6 3-position switch: 1000 / 1500 / 2000 us.
constexpr uint16_t kSwitchLowMax = 1250;
constexpr uint16_t kSwitchHighMin = 1750;

// CH5 above about 1.7 ms hands the driver to the receiver
// (docs/build/03a-drive-wiring.md 4).
constexpr uint16_t kMuxSelHighMin = 1700;

bool InRange(uint16_t us) { return us >= kUsMin && us <= kUsMax; }

float Stick(uint16_t us) {
  float v = ((float)us - kUsCentre) / kUsHalfSpan;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  if (v > -kStickDeadband && v < kStickDeadband) v = 0.0f;
  return v;
}

}  // namespace

bool IbusParser::Feed(uint8_t b) {
  if (len_ == 0) {
    if (b == kHeader0) buf_[len_++] = b;
    return false;
  }
  if (len_ == 1) {
    if (b == kHeader1) {
      buf_[len_++] = b;
    } else {
      // Resynchronise: this byte may itself be the start of a frame.
      len_ = (b == kHeader0) ? 1 : 0;
    }
    return false;
  }

  buf_[len_++] = b;
  if (len_ < kFrameLen) return false;
  len_ = 0;

  uint16_t sum = 0xFFFF;
  for (int i = 0; i < kFrameLen - 2; ++i) sum = (uint16_t)(sum - buf_[i]);
  const uint16_t rx = (uint16_t)(buf_[kFrameLen - 2] | (buf_[kFrameLen - 1] << 8));
  if (sum != rx) {
    ++bad_frames_;
    return false;
  }

  for (int i = 0; i < kChannels; ++i) {
    ch_[i] = (uint16_t)(buf_[2 + 2 * i] | ((buf_[3 + 2 * i] & 0x0F) << 8));
  }
  return true;
}

RcCommand DecodeRc(const uint16_t ch[IbusParser::kChannels]) {
  RcCommand out;
  const uint16_t steer = ch[rc_ch::kSteer];
  const uint16_t throttle = ch[rc_ch::kThrottle];
  const uint16_t mode = ch[rc_ch::kMode];
  const uint16_t sel = ch[rc_ch::kMuxSel];

  if (!InRange(steer) || !InRange(throttle) || !InRange(mode)) return out;

  out.valid = true;
  // Stick RIGHT gives CH1 above 1500 and must turn RIGHT, which is a NEGATIVE
  // angle. Layer B sends the same CH1 straight to RC1, so this is the same
  // rule the wiring follows (pulse.h).
  out.steer = -Stick(steer);
  out.throttle = Stick(throttle);

  if (mode <= kSwitchLowMax) {
    out.sw = RcSwitch::kKill;
  } else if (mode >= kSwitchHighMin) {
    out.sw = RcSwitch::kAuto;
  } else {
    out.sw = RcSwitch::kManual;
  }
  out.mux_to_receiver = InRange(sel) && sel >= kMuxSelHighMin;
  return out;
}

}  // namespace dbw
