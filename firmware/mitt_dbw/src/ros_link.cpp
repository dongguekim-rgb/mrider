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

#include "ros_link.h"

#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>
#include <mitt_msgs/msg/dbw_command.h>
#include <mitt_msgs/msg/dbw_status.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

#include "config.h"
#include "control.h"
#include "dbw_types.h"
#include "persist.h"

namespace ros_link {

// dbw_core has no ROS in it, so it carries its own copy of the message's
// constants. These make a mismatch a compile error, not a wrong bit on a bag.
static_assert(dbw::fault::kEncoder == mitt_msgs__msg__DbwStatus__FAULT_ENCODER, "DbwStatus");
static_assert(dbw::fault::kSetpointStale == mitt_msgs__msg__DbwStatus__FAULT_SETPOINT_STALE,
              "DbwStatus");
static_assert(dbw::fault::kAtLimit == mitt_msgs__msg__DbwStatus__FAULT_AT_LIMIT, "DbwStatus");
static_assert(dbw::fault::kStall == mitt_msgs__msg__DbwStatus__FAULT_STALL, "DbwStatus");
static_assert(dbw::fault::kRcLoss == mitt_msgs__msg__DbwStatus__FAULT_RC_LOSS, "DbwStatus");
static_assert(dbw::fault::kOverCurrent == mitt_msgs__msg__DbwStatus__FAULT_OVER_CURRENT,
              "DbwStatus");
// fault::kEstop (64) is deliberately NOT asserted: a team fork must not change
// mitt_msgs, so the bit has to work whether or not its mitt_msgs carries the
// FAULT_ESTOP constant yet. The wire format is the same uint16 either way.
static_assert((uint8_t)dbw::Mode::kEstop == mitt_msgs__msg__DbwStatus__MODE_ESTOP, "DbwStatus");
static_assert((uint8_t)dbw::Mode::kManualRc == mitt_msgs__msg__DbwStatus__MODE_MANUAL_RC,
              "DbwStatus");
static_assert((uint8_t)dbw::Mode::kAutonomous == mitt_msgs__msg__DbwStatus__MODE_AUTONOMOUS,
              "DbwStatus");

namespace {

// ============================================================ parameters

struct FloatParam {
  const char *name;
  float dbw::Params::*field;
  bool calibration;  // changes what an angle MEANS: ESTOP only
  float lo;
  float hi;
};

const FloatParam kFloatParams[] = {
    {"steer_rad_per_count", &dbw::Params::steer_rad_per_count, true, -0.01f, 0.01f},
    {"steer_limit_rad", &dbw::Params::steer_limit_rad, true, 0.05f, 0.6f},
    {"steer_stop_rad", &dbw::Params::steer_stop_rad, true, 0.05f, 0.8f},
    {"steer_kp", &dbw::Params::steer_kp, false, 0.0f, 50.0f},
    {"steer_ki", &dbw::Params::steer_ki, false, 0.0f, 50.0f},
    {"steer_kd", &dbw::Params::steer_kd, false, 0.0f, 5.0f},
    {"steer_i_max", &dbw::Params::steer_i_max, false, 0.0f, 1.0f},
    {"steer_deadband_rad", &dbw::Params::steer_deadband_rad, false, 0.0f, 0.05f},
    {"steer_effort_max", &dbw::Params::steer_effort_max, false, 0.0f, 1.0f},
    {"drive_m_per_tick", &dbw::Params::drive_m_per_tick, true, 0.0001f, 0.2f},
    {"drive_effort_per_mps", &dbw::Params::drive_effort_per_mps, false, 0.0f, 5.0f},
    {"drive_effort_max", &dbw::Params::drive_effort_max, false, 0.0f, 1.0f},
    {"speed_cap_mps", &dbw::Params::speed_cap_mps, false, 0.0f, 3.0f},
    {"rc_speed_max_mps", &dbw::Params::rc_speed_max_mps, false, 0.0f, 3.0f},
    {"accel_effort_per_s", &dbw::Params::accel_effort_per_s, false, 0.05f, 10.0f},
    {"decel_effort_per_s", &dbw::Params::decel_effort_per_s, false, 0.05f, 20.0f},
    {"reversal_speed_mps", &dbw::Params::reversal_speed_mps, false, 0.0f, 1.0f},
};
constexpr size_t kNumFloat = sizeof(kFloatParams) / sizeof(kFloatParams[0]);
constexpr const char *kZeroParam = "steer_zero_counts";
constexpr size_t kNumParams = kNumFloat + 1;

// ============================================================ state

enum class Link { kWaiting, kAvailable, kConnected, kDisconnected };
Link g_link = Link::kWaiting;
uint32_t g_link_ms = 0;

rcl_allocator_t g_alloc;
rclc_support_t g_support;
rcl_node_t g_node;
rclc_executor_t g_exec;
rcl_publisher_t g_status_pub;
rcl_publisher_t g_imu_pub;
rcl_publisher_t g_event_pub;
rcl_subscription_t g_cmd_sub;
rcl_service_t g_zero_srv;
rcl_service_t g_save_srv;
#ifdef MITT_HANG_TEST
rcl_service_t g_hang_srv;
#endif
rclc_parameter_server_t g_params;
bool g_support_ok = false;    // rclc_support_init succeeded this session
bool g_params_ok = false;     // the parameter server exists this session
bool g_params_ready = false;  // false while the server is being filled

mitt_msgs__msg__DbwCommand g_cmd_msg;
mitt_msgs__msg__DbwStatus g_status_msg;
sensor_msgs__msg__Imu g_imu_msg;
std_msgs__msg__String g_event_msg;
char g_event_buf[192];

std_srvs__srv__Trigger_Request g_zero_req, g_save_req;
std_srvs__srv__Trigger_Response g_zero_res, g_save_res;
char g_zero_text[96], g_save_text[96];
#ifdef MITT_HANG_TEST
std_srvs__srv__Trigger_Request g_hang_req;
std_srvs__srv__Trigger_Response g_hang_res;
char g_hang_text[64];
#endif

uint32_t g_last_status_ms = 0;
uint32_t g_last_diag_ms = 0;
uint32_t g_last_sync_ms = 0;
uint32_t g_last_ping_ms = 0;
uint32_t g_stats_since_ms = 0;
bool g_imu_pending = false;
imu::Sample g_imu_sample;

// ============================================================ helpers

// rcl marks its returns warn_unused_result, which a (void) cast does not
// silence. These calls are best-effort by design: a lost status frame is
// replaced 10 ms later, and teardown of a dead session cannot fail usefully.
inline void BestEffort(rcl_ret_t) {}

void SetText(rosidl_runtime_c__String *s, char *buf, size_t cap, const char *text) {
  snprintf(buf, cap, "%s", text);
  s->data = buf;
  s->size = strlen(buf);
  s->capacity = cap;
}

void Publish(const char *text) {
  if (g_link != Link::kConnected) return;
  SetText(&g_event_msg.data, g_event_buf, sizeof(g_event_buf), text);
  BestEffort(rcl_publish(&g_event_pub, &g_event_msg, NULL));
}

// Laptop-referenced time, micro-ROS session sync (calibration.md 6), for a
// moment that was `age_us` ago on the Teensy clock.
void StampAgo(builtin_interfaces__msg__Time *t, uint32_t age_us) {
  const int64_t ns = rmw_uros_epoch_nanos() - (int64_t)age_us * 1000;
  t->sec = (int32_t)(ns / 1000000000LL);
  t->nanosec = (uint32_t)(ns % 1000000000LL);
}

bool SafeToCalibrate(char *why, size_t n) {
  const control::Snapshot s = control::Take();
  if (s.mode != (uint8_t)dbw::Mode::kEstop || s.relay_dbw) {
    snprintf(why, n, "refused: only in ESTOP with the relay MUX released (mode %u)", s.mode);
    return false;
  }
  return true;
}

// ============================================================ callbacks

void OnCommand(const void *msgin) {
  const auto *m = static_cast<const mitt_msgs__msg__DbwCommand *>(msgin);
  // A NaN is not a command. Dropped, so staleness does its job.
  if (!isfinite(m->steering_angle) || !isfinite(m->speed)) return;
  control::OnCommand(m->steering_angle, m->speed);
}

bool OnParam(const Parameter *old_p, const Parameter *new_p, void *) {
  if (old_p == NULL || new_p == NULL) return false;  // no create, no delete
  if (!g_params_ready) return true;                  // our own initial fill

  const char *name = new_p->name.data;
  dbw::Params p = control::GetParams();
  char why[96];

  if (strcmp(name, kZeroParam) == 0) {
    if (new_p->value.type != RCLC_PARAMETER_INT) return false;
    const int64_t v = new_p->value.integer_value;
    if (v < -1 || v > 4095) return false;
    if (!SafeToCalibrate(why, sizeof(why))) return false;
    p.steer_zero_counts = (int32_t)v;
    control::SetParams(p);
    return true;
  }

  for (const FloatParam &f : kFloatParams) {
    if (strcmp(name, f.name) != 0) continue;
    if (new_p->value.type != RCLC_PARAMETER_DOUBLE) return false;
    const float v = (float)new_p->value.double_value;
    if (!isfinite(v) || v < f.lo || v > f.hi) return false;
    if (f.calibration && !SafeToCalibrate(why, sizeof(why))) return false;
    p.*(f.field) = v;
    if (p.steer_stop_rad < p.steer_limit_rad) return false;  // stop inside the range
    if (p.steer_rad_per_count == 0.0f) return false;
    control::SetParams(p);
    return true;
  }
  return false;
}

void OnZero(const void *, void *res_v) {
  auto *res = static_cast<std_srvs__srv__Trigger_Response *>(res_v);
  char text[96];
  const control::Snapshot s = control::Take();
  if (!SafeToCalibrate(text, sizeof(text))) {
    res->success = false;
  } else if (!s.raw_ok) {
    snprintf(text, sizeof(text), "refused: AS5600 not read, or magnet not detected");
    res->success = false;
  } else if (rclc_parameter_set_int(&g_params, kZeroParam, s.raw_counts) != RCL_RET_OK) {
    snprintf(text, sizeof(text), "refused by the parameter server");
    res->success = false;
  } else {
    snprintf(text, sizeof(text), "c0 = %u counts. Not yet saved: call /mitt_dbw/save_params",
             s.raw_counts);
    res->success = true;
  }
  SetText(&res->message, g_zero_text, sizeof(g_zero_text), text);
}

void OnSave(const void *, void *res_v) {
  auto *res = static_cast<std_srvs__srv__Trigger_Response *>(res_v);
  char text[96];
  if (!SafeToCalibrate(text, sizeof(text))) {
    res->success = false;
  } else {
    persist::Save(control::GetParams());
    snprintf(text, sizeof(text), "saved to EEPROM");
    res->success = true;
  }
  SetText(&res->message, g_save_text, sizeof(g_save_text), text);
}

#ifdef MITT_HANG_TEST
void OnHang(const void *, void *res_v) {
  auto *res = static_cast<std_srvs__srv__Trigger_Response *>(res_v);
  control::RequestHang();
  res->success = true;
  SetText(&res->message, g_hang_text, sizeof(g_hang_text),
          "control loop hanging now; time the motor stopping");
}
#endif

// ============================================================ entities

#define RCCHECK(x)                    \
  do {                                \
    if ((x) != RCL_RET_OK) return false; \
  } while (0)

bool FillParams() {
  rclc_parameter_options_t opts = {};
  opts.notify_changed_over_dds = false;
  opts.max_params = kNumParams;
  opts.allow_undeclared_parameters = false;
  opts.low_mem_mode = false;
  RCCHECK(rclc_parameter_server_init_with_option(&g_params, &g_node, &opts));
  g_params_ok = true;

  const dbw::Params p = control::GetParams();
  g_params_ready = false;
  RCCHECK(rclc_add_parameter(&g_params, kZeroParam, RCLC_PARAMETER_INT));
  RCCHECK(rclc_parameter_set_int(&g_params, kZeroParam, p.steer_zero_counts));
  for (const FloatParam &f : kFloatParams) {
    RCCHECK(rclc_add_parameter(&g_params, f.name, RCLC_PARAMETER_DOUBLE));
    RCCHECK(rclc_parameter_set_double(&g_params, f.name, (double)(p.*(f.field))));
  }
  return true;
}

bool Create() {
  g_alloc = rcl_get_default_allocator();

  // Every handle starts zero-initialised, so a Create() that fails half way
  // leaves Destroy() finalising only what exists - never a handle from the
  // last session twice.
  g_node = rcl_get_zero_initialized_node();
  g_status_pub = rcl_get_zero_initialized_publisher();
  g_imu_pub = rcl_get_zero_initialized_publisher();
  g_event_pub = rcl_get_zero_initialized_publisher();
  g_cmd_sub = rcl_get_zero_initialized_subscription();
  g_zero_srv = rcl_get_zero_initialized_service();
  g_save_srv = rcl_get_zero_initialized_service();
#ifdef MITT_HANG_TEST
  g_hang_srv = rcl_get_zero_initialized_service();
#endif
  g_exec = rclc_executor_get_zero_initialized_executor();

  rcl_init_options_t init = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init, g_alloc));
  RCCHECK(rcl_init_options_set_domain_id(&init, cfg::kRosDomain));
  RCCHECK(rclc_support_init_with_options(&g_support, 0, NULL, &init, &g_alloc));
  g_support_ok = true;
  RCCHECK(rclc_node_init_default(&g_node, "mitt_dbw", "", &g_support));

  RCCHECK(rclc_publisher_init_best_effort(
      &g_status_pub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(mitt_msgs, msg, DbwStatus),
      "/mitt/dbw/status"));
  RCCHECK(rclc_publisher_init_best_effort(
      &g_imu_pub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
  RCCHECK(rclc_publisher_init_default(
      &g_event_pub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/mitt/dbw/event"));
  RCCHECK(rclc_subscription_init_best_effort(
      &g_cmd_sub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(mitt_msgs, msg, DbwCommand),
      "/mitt/dbw/command"));
  RCCHECK(rclc_service_init_default(&g_zero_srv, &g_node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "/mitt_dbw/zero_steering"));
  RCCHECK(rclc_service_init_default(&g_save_srv, &g_node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "/mitt_dbw/save_params"));
#ifdef MITT_HANG_TEST
  RCCHECK(rclc_service_init_default(&g_hang_srv, &g_node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "/mitt_dbw/test_hang"));
#endif
  if (!FillParams()) return false;

  size_t handles = 1 + 2 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
#ifdef MITT_HANG_TEST
  handles += 1;
#endif
  RCCHECK(rclc_executor_init(&g_exec, &g_support.context, handles, &g_alloc));
  RCCHECK(rclc_executor_add_subscription(&g_exec, &g_cmd_sub, &g_cmd_msg, OnCommand,
                                         ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&g_exec, &g_zero_srv, &g_zero_req, &g_zero_res, OnZero));
  RCCHECK(rclc_executor_add_service(&g_exec, &g_save_srv, &g_save_req, &g_save_res, OnSave));
#ifdef MITT_HANG_TEST
  RCCHECK(rclc_executor_add_service(&g_exec, &g_hang_srv, &g_hang_req, &g_hang_res, OnHang));
#endif
  RCCHECK(rclc_executor_add_parameter_server_with_context(&g_exec, &g_params, OnParam, NULL));
  g_params_ready = true;

  // calibration.md 6: "Verify the sync is actually established at startup
  // rather than assuming it - log the reported offset once per session."
  const bool synced = rmw_uros_sync_session(200) == RMW_RET_OK;
  g_last_sync_ms = millis();
  g_link = Link::kConnected;  // so Publish() goes out
  char text[128];
  if (synced) {
    snprintf(text, sizeof(text),
             "session up: vehicle %d, domain %d, time synced (laptop epoch - teensy uptime = %.3f s)",
             cfg::kVehicle, cfg::kRosDomain,
             (double)(rmw_uros_epoch_millis() - (int64_t)millis()) / 1000.0);
  } else {
    snprintf(text, sizeof(text),
             "session up: vehicle %d, domain %d, TIME SYNC FAILED - stamps are not laptop time",
             cfg::kVehicle, cfg::kRosDomain);
  }
  Publish(text);
  return true;
}

void Destroy() {
  if (!g_support_ok) return;  // nothing was created
  rmw_context_t *ctx = rcl_context_get_rmw_context(&g_support.context);
  if (ctx != NULL) (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

  g_params_ready = false;
  BestEffort(rclc_executor_fini(&g_exec));
  if (g_params_ok) BestEffort(rclc_parameter_server_fini(&g_params, &g_node));
  g_params_ok = false;
  BestEffort(rcl_service_fini(&g_zero_srv, &g_node));
  BestEffort(rcl_service_fini(&g_save_srv, &g_node));
#ifdef MITT_HANG_TEST
  BestEffort(rcl_service_fini(&g_hang_srv, &g_node));
#endif
  BestEffort(rcl_subscription_fini(&g_cmd_sub, &g_node));
  BestEffort(rcl_publisher_fini(&g_event_pub, &g_node));
  BestEffort(rcl_publisher_fini(&g_imu_pub, &g_node));
  BestEffort(rcl_publisher_fini(&g_status_pub, &g_node));
  BestEffort(rcl_node_fini(&g_node));
  BestEffort(rclc_support_fini(&g_support));
  g_support_ok = false;
}

// ============================================================ publishing

void PublishStatus() {
  const control::Snapshot s = control::Take();
  StampAgo(&g_status_msg.stamp, (uint32_t)(micros() - s.meas_us));
  g_status_msg.steering_angle = s.angle_rad;
  g_status_msg.steering_setpoint = s.setpoint_rad;
  g_status_msg.wheel_speed = s.wheel_speed_mps;
  g_status_msg.drive_ticks = s.drive_ticks;
  g_status_msg.mode = s.mode;
  g_status_msg.faults = s.faults;
  BestEffort(rcl_publish(&g_status_pub, &g_status_msg, NULL));
}

void PublishImu() {
  const imu::Sample &s = g_imu_sample;
  sensor_msgs__msg__Imu &m = g_imu_msg;
  StampAgo(&m.header.stamp, (uint32_t)(micros() - s.t_us));
  m.orientation.x = s.qx;
  m.orientation.y = s.qy;
  m.orientation.z = s.qz;
  m.orientation.w = s.qw;
  m.angular_velocity.x = s.gx;
  m.angular_velocity.y = s.gy;
  m.angular_velocity.z = s.gz;
  m.linear_acceleration.x = s.ax;
  m.linear_acceleration.y = s.ay;
  m.linear_acceleration.z = s.az;
  BestEffort(rcl_publish(&g_imu_pub, &m, NULL));
}

void PublishDiag() {
  const uint32_t now = millis();
  control::LoopStats st = control::TakeStats();
  const float secs = (float)(now - g_stats_since_ms) / 1000.0f;
  g_stats_since_ms = now;
  st.hz = secs > 0.0f ? (float)st.ticks / secs : 0.0f;
  const control::Snapshot s = control::Take();

  static const char *const kModes[] = {"ESTOP", "MANUAL_RC", "AUTONOMOUS"};
  char text[192];
  snprintf(text, sizeof(text),
           "loop %.1f Hz (period %lu..%lu us, exec max %lu us, overruns %lu) | %s faults 0x%04X | "
           "raw %u%s | mux %s | ibus bad %lu | %s",
           (double)st.hz, (unsigned long)st.min_period_us, (unsigned long)st.max_period_us,
           (unsigned long)st.max_exec_us, (unsigned long)st.overruns,
           s.mode < 3 ? kModes[s.mode] : "?", s.faults, s.raw_counts, s.raw_ok ? "" : " (bad)",
           s.mux_to_receiver ? "RECEIVER" : "teensy", (unsigned long)st.ibus_bad_frames,
           imu::Describe());
  Publish(text);
}

void InitMessages() {
  memset(&g_status_msg, 0, sizeof(g_status_msg));
  memset(&g_cmd_msg, 0, sizeof(g_cmd_msg));
  memset(&g_event_msg, 0, sizeof(g_event_msg));
  memset(&g_zero_res, 0, sizeof(g_zero_res));
  memset(&g_save_res, 0, sizeof(g_save_res));

  memset(&g_imu_msg, 0, sizeof(g_imu_msg));
  static char frame[] = "imu_link";
  g_imu_msg.header.frame_id.data = frame;
  g_imu_msg.header.frame_id.size = strlen(frame);
  g_imu_msg.header.frame_id.capacity = sizeof(frame);
  // Diagonal covariances. PLACEHOLDERS from the BNO085 datasheet's typical
  // figures, not measured on this vehicle: orientation ~2.9 deg, gyro noise
  // ~0.01 rad/s, accel ~0.1 m/s^2. calibration.md 5 is where they get real.
  for (int i = 0; i < 3; ++i) {
    g_imu_msg.orientation_covariance[i * 4] = 0.0025;
    g_imu_msg.angular_velocity_covariance[i * 4] = 1.0e-4;
    g_imu_msg.linear_acceleration_covariance[i * 4] = 0.01;
  }
}

}  // namespace

void Begin() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  InitMessages();
}

void OnImu(const imu::Sample &s) {
  g_imu_sample = s;
  g_imu_pending = true;
}

void Spin() {
  const uint32_t now = millis();

  switch (g_link) {
    case Link::kWaiting:
      if ((uint32_t)(now - g_link_ms) >= 500) {
        g_link_ms = now;
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) g_link = Link::kAvailable;
      }
      return;

    case Link::kAvailable:
      if (Create()) {
        g_link = Link::kConnected;
        g_last_ping_ms = g_last_status_ms = g_last_diag_ms = g_stats_since_ms = now;
      } else {
        Destroy();
        g_link = Link::kWaiting;
        g_link_ms = now;
      }
      return;

    case Link::kDisconnected:
      Destroy();
      g_link = Link::kWaiting;
      g_link_ms = now;
      return;

    case Link::kConnected:
      break;
  }

  if ((uint32_t)(now - g_last_ping_ms) >= 1000) {
    g_last_ping_ms = now;
    if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
      g_link = Link::kDisconnected;
      return;
    }
  }

  BestEffort(rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(0)));

  if ((uint32_t)(now - g_last_status_ms) >= 1000 / cfg::kStatusHz) {
    g_last_status_ms += 1000 / cfg::kStatusHz;
    if ((uint32_t)(now - g_last_status_ms) > 100) g_last_status_ms = now;  // fell behind
    PublishStatus();
  }

  // 04-firmware.md: on a fault, stop publishing - silence, not a stale sample.
  if (g_imu_pending) {
    g_imu_pending = false;
    if (imu::Healthy()) PublishImu();
  }

  const char *ev;
  while (control::PopEvent(&ev)) Publish(ev);

  if ((uint32_t)(now - g_last_diag_ms) >= 5000) {
    g_last_diag_ms = now;
    PublishDiag();
  }

  if ((uint32_t)(now - g_last_sync_ms) >= 60000) {
    g_last_sync_ms = now;
    (void)rmw_uros_sync_session(50);
  }
}

}  // namespace ros_link
