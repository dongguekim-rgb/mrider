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
// Params in EEPROM, so the steering centre survives a reboot
// (docs/design/calibration.md 1.1 step 4 - the fix for B-MROVER's arbitrary
// boot centre, finding F4).
//
// A record that does not check out - never written, a different firmware's
// layout, a torn write - is ignored and the defaults are used, and the
// defaults have steer_zero_counts = -1, so the vehicle will not arm until it
// is zeroed again. A bad record can cost a calibration; it cannot drive.

#pragma once

#include "dbw_types.h"

namespace persist {

// True if a valid record was found and copied into *out.
bool Load(dbw::Params *out);

// Write. Stops every interrupt for as long as flash erase takes - call only
// with the vehicle in ESTOP and both lines at neutral (ros_link enforces it).
void Save(const dbw::Params &p);

}  // namespace persist
