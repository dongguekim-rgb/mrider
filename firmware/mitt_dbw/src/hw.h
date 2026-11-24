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
// The thin layer over Teensy peripherals: the angle sensor, the two pulse
// lines, the relay gate, and the watchdog. Nothing here decides anything.

#pragma once

#include <stdint.h>

#include "sensors.h"

namespace hw {

// --- AS5600 on Wire (pins 18/19) ---------------------------------------------
void AngleBegin();
dbw::AngleSensor::Sample AngleRead();

// --- servo pulses to the signal MUX, and the relay gate ----------------------
// PulsesBegin puts both lines at exactly 1500 us before anything else runs.
void PulsesBegin();
void PulsesWrite(float steer_us, float drive_us);
void RelayGate(bool dbw);

// --- RTWDOG (WDOG3), 32 kHz clock --------------------------------------------
void WatchdogBegin(uint32_t timeout_ms);
void WatchdogSetTimeout(uint32_t timeout_ms);
void WatchdogFeed();
// True if the last reset was this watchdog firing. Read once, at boot.
bool ResetWasWatchdog();

}  // namespace hw
