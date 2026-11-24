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

#include "control.h"

#include <Arduino.h>
#include <IntervalTimer.h>
#include <QuadEncoder.h>

#include "config.h"
#include "dbw_limits.h"
#include "hw.h"
#include "ibus.h"
#include "pulse.h"
#include "sensors.h"
#include "supervisor.h"

namespace control {

namespace {

// Below USB (112) and the UARTs (64) so neither starves; above the main loop,
// which is not an interrupt at all.
constexpr uint8_t kTickPriority = 160;
constexpr uint32_t kPeriodUs = 1000000u / dbw::limits::kLoopHz;

IntervalTimer g_timer;
// channel 1, A = pin 0, B = pin 1, 0 = no internal pull-up: the DRV5013 is
// open-drain and the signal board carries 10 k pull-ups (bench quad_count).
QuadEncoder g_enc(1, cfg::kPinEncA, cfg::kPinEncB, 0);

dbw::Supervisor g_sup;
dbw::AngleSensor g_angle;
dbw::WheelSpeed g_wheel;
dbw::IbusParser g_ibus;
DMAMEM uint8_t g_ibus_rx_buf[256];

// Shared with the main loop. The main loop only ever touches these with
// interrupts off; the tick cannot be interrupted by the main loop at all.
dbw::Params g_params;
volatile float g_cmd_steer = 0.0f;
volatile float g_cmd_speed = 0.0f;
volatile uint32_t g_cmd_rx_ms = 0;
volatile bool g_cmd_seen = false;
volatile uint32_t g_heartbeat_ms = 0;
Snapshot g_snap;
LoopStats g_stats;

// i-BUS state, tick-only.
dbw::RcCommand g_rc;
uint32_t g_rc_rx_ms = 0;
bool g_rc_seen = false;

// Events: a small ring of string literals, tick -> main.
constexpr int kEvents = 16;
const char *volatile g_events[kEvents];
volatile uint8_t g_ev_head = 0;
volatile uint8_t g_ev_tail = 0;

uint32_t g_last_tick_us = 0;
bool g_first_tick = true;

#ifdef MITT_HANG_TEST
volatile bool g_hang = false;
#endif

void PushEvent(const char *e) {
  const uint8_t next = (uint8_t)((g_ev_head + 1) % kEvents);
  if (next == g_ev_tail) return;  // full: drop the newest, keep the cause
  g_events[g_ev_head] = e;
  g_ev_head = next;
}

void Tick() {
  const uint32_t t0 = micros();
  const uint32_t now_ms = millis();

  // ---- period statistics ("Loop rate >= 200 Hz, 실측") ----------------------
  float dt_s = (float)kPeriodUs * 1e-6f;
  if (!g_first_tick) {
    const uint32_t period = t0 - g_last_tick_us;
    dt_s = (float)period * 1e-6f;
    if (g_stats.ticks == 0 || period < g_stats.min_period_us) g_stats.min_period_us = period;
    if (period > g_stats.max_period_us) g_stats.max_period_us = period;
    if (period > kPeriodUs + kPeriodUs / 2) ++g_stats.overruns;
    ++g_stats.ticks;
  }
  g_first_tick = false;
  g_last_tick_us = t0;
  if (dt_s > 0.05f) dt_s = 0.05f;

  const dbw::Params &p = g_params;

  // ---- sense ----------------------------------------------------------------
  const dbw::AngleSensor::Sample s = hw::AngleRead();
  const uint32_t meas_us = micros();
  g_angle.Update(s, dt_s, p);

  const int32_t ticks = g_enc.read();
  g_wheel.Update(ticks, now_ms, p.drive_m_per_tick);

  while (Serial3.available() > 0) {
    if (g_ibus.Feed((uint8_t)Serial3.read())) {
      uint16_t ch[dbw::IbusParser::kChannels];
      for (int i = 0; i < dbw::IbusParser::kChannels; ++i) ch[i] = g_ibus.channel(i);
      const dbw::RcCommand rc = dbw::DecodeRc(ch);
      if (rc.valid) {
        g_rc = rc;
        g_rc_rx_ms = now_ms;
        g_rc_seen = true;
      }
    }
  }

  const bool estop_high = digitalReadFast(cfg::kPinEstopSense) == HIGH;

  // ---- decide -----------------------------------------------------------------
  dbw::Inputs in;
  in.now_ms = now_ms;
  in.dt_s = dt_s;
  in.estop_high = estop_high;
  in.angle_ok = g_angle.ok();
  in.angle_rad = g_angle.angle_rad();
  in.angle_rate_rad_s = g_angle.rate_rad_s();
  in.steer_zeroed = p.steer_zero_counts >= 0;
  in.rc_age_ms = g_rc_seen ? (uint32_t)(now_ms - g_rc_rx_ms) : UINT32_MAX;
  in.rc_switch = g_rc.sw;
  in.rc_steer = g_rc.steer;
  in.rc_throttle = g_rc.throttle;
  in.cmd_age_ms = g_cmd_seen ? (uint32_t)(now_ms - g_cmd_rx_ms) : UINT32_MAX;
  in.cmd_steer_rad = g_cmd_steer;
  in.cmd_speed_mps = g_cmd_speed;
  in.wheel_speed_mps = g_wheel.speed_mps();

  const dbw::Outputs out = g_sup.Step(in, p);

  // ---- act ------------------------------------------------------------------
  hw::PulsesWrite(dbw::SteerPulseUs(out.steer_effort), dbw::DrivePulseUs(out.drive_effort));
  hw::RelayGate(out.relay_dbw);

  if (out.event) PushEvent(out.event);

  Snapshot &sn = g_snap;
  sn.meas_us = meas_us;
  sn.angle_rad = g_angle.angle_rad();
  sn.setpoint_rad = out.setpoint_rad;
  sn.wheel_speed_mps = g_wheel.speed_mps();
  sn.drive_ticks = ticks;
  sn.mode = (uint8_t)out.mode;
  sn.faults = out.faults;
  sn.raw_counts = g_angle.raw();
  sn.raw_ok = s.read_ok && s.magnet_ok;
  sn.relay_dbw = out.relay_dbw;
  sn.mux_to_receiver = g_rc_seen && g_rc.mux_to_receiver;

  const uint32_t exec = micros() - t0;
  if (exec > g_stats.max_exec_us) g_stats.max_exec_us = exec;

#ifdef MITT_HANG_TEST
  if (g_hang) {
    // The hang under test: the pulse timers run on with the last width, and
    // nothing feeds the watchdog. 04-firmware.md 4.4.
    while (true) {
    }
  }
#endif

  // ---- feed, last -------------------------------------------------------------
  // Only after a tick that read the sensor and wrote both outputs, and only
  // while the main loop is still coming round.
  if ((uint32_t)(now_ms - g_heartbeat_ms) < cfg::kMainLoopStallMs) hw::WatchdogFeed();
}

}  // namespace

void Begin(const dbw::Params &p) {
  g_params = p;
  g_heartbeat_ms = millis();

  pinMode(cfg::kPinEstopSense, INPUT);  // the divider drives it; no pull
  hw::AngleBegin();

  g_enc.setInitConfig();
  g_enc.init();

  Serial3.begin(115200);  // 8N1, not inverted: i-BUS
  Serial3.addMemoryForRead(g_ibus_rx_buf, sizeof(g_ibus_rx_buf));

  g_timer.priority(kTickPriority);
  g_timer.begin(Tick, kPeriodUs);
}

void SetParams(const dbw::Params &p) {
  noInterrupts();
  g_params = p;
  interrupts();
}

dbw::Params GetParams() {
  noInterrupts();
  const dbw::Params p = g_params;
  interrupts();
  return p;
}

void OnCommand(float steer_rad, float speed_mps) {
  const uint32_t now = millis();
  noInterrupts();
  g_cmd_steer = steer_rad;
  g_cmd_speed = speed_mps;
  g_cmd_rx_ms = now;
  g_cmd_seen = true;
  interrupts();
}

void Heartbeat() { g_heartbeat_ms = millis(); }

void PostEvent(const char *literal) {
  noInterrupts();
  PushEvent(literal);
  interrupts();
}

Snapshot Take() {
  noInterrupts();
  const Snapshot s = g_snap;
  interrupts();
  return s;
}

bool PopEvent(const char **literal) {
  noInterrupts();
  const bool any = g_ev_tail != g_ev_head;
  if (any) {
    *literal = g_events[g_ev_tail];
    g_ev_tail = (uint8_t)((g_ev_tail + 1) % kEvents);
  }
  interrupts();
  return any;
}

LoopStats TakeStats() {
  noInterrupts();
  LoopStats s = g_stats;
  g_stats = LoopStats();
  interrupts();
  s.ibus_bad_frames = g_ibus.bad_frames();
  return s;
}

#ifdef MITT_HANG_TEST
void RequestHang() { g_hang = true; }
#endif

}  // namespace control
