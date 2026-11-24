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
// FlySky i-BUS, as the FS-iA6B sends it: 115200 8N1, NOT inverted.
//
//   0x20 0x40  ch1_lo ch1_hi ... ch14_lo ch14_hi  sum_lo sum_hi     32 bytes
//   checksum = 0xFFFF - (sum of the first 30 bytes)
//
// Each channel is microseconds, 1000..2000. The top nibble of the high byte
// is used by newer FlySky firmware for other things; it is masked off.

#pragma once

#include <stdint.h>

#include "dbw_types.h"

namespace dbw {

class IbusParser {
 public:
  static constexpr int kChannels = 14;
  static constexpr int kFrameLen = 32;

  // Feed one byte. True when it completed a frame whose checksum is right;
  // channel() then holds that frame. A bad frame is dropped without a trace
  // other than bad_frames().
  bool Feed(uint8_t b);

  uint16_t channel(int i) const { return ch_[i]; }
  uint32_t bad_frames() const { return bad_frames_; }

 private:
  uint8_t buf_[kFrameLen] = {};
  int len_ = 0;
  uint16_t ch_[kChannels] = {};
  uint32_t bad_frames_ = 0;
};

// Which transmitter channel does what. CH1, CH2 and CH5 are fixed by the
// wiring - the same CH1/CH2 also drive the signal MUX's SLAVE inputs, and CH5
// is its SEL (docs/build/03-electrical.md 3.5.1). CH6 is this firmware's
// choice: a 3-position switch (SwC on an FS-i6) assigned to it.
namespace rc_ch {
constexpr int kSteer = 0;     // CH1
constexpr int kThrottle = 1;  // CH2
constexpr int kMuxSel = 4;    // CH5 - read for the record, never acted on
constexpr int kMode = 5;      // CH6 - KILL / MANUAL / AUTO
}  // namespace rc_ch

struct RcCommand {
  bool valid = false;          // every channel used here was in range
  RcSwitch sw = RcSwitch::kUnknown;
  float steer = 0.0f;          // -1..1, + = left
  float throttle = 0.0f;       // -1..1, + = forward
  bool mux_to_receiver = false;  // CH5 high: the hardware MUX is passing RC
};

// Channels in microseconds -> what they mean. A frame with any used channel
// outside 900..2100 us is treated as invalid rather than clamped: that is a
// receiver saying something other than a stick position.
RcCommand DecodeRc(const uint16_t ch[IbusParser::kChannels]);

}  // namespace dbw
