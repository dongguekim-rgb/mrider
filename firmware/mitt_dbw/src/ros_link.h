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
// micro-ROS over USB serial: the vehicle's whole ROS interface.
//
//   node     /mitt_dbw                 domain 10 + MITT_VEHICLE
//   sub      /mitt/dbw/command         mitt_msgs/DbwCommand   best effort
//   pub      /mitt/dbw/status          mitt_msgs/DbwStatus    best effort, 100 Hz
//   pub      /imu/data                 sensor_msgs/Imu        best effort, ~200 Hz
//   pub      /mitt/dbw/event           std_msgs/String        reliable, on change + every 5 s
//   srv      /mitt_dbw/zero_steering   std_srvs/Trigger       ESTOP only
//   srv      /mitt_dbw/save_params     std_srvs/Trigger       ESTOP only
//   srv      /mitt_dbw/test_hang       std_srvs/Trigger       hang_test build only
//   params   steer_kp, steer_zero_counts, ...  (ros2 param list /mitt_dbw)
//
// BEST EFFORT. A subscriber on the laptop must use SensorDataQoS (or any
// best-effort profile) to receive status and IMU; a reliable subscriber
// matches nothing and hears nothing. `ros2 topic echo` and `ros2 topic hz`
// adapt on their own.
//
// The session comes and goes with the agent: unplug, restart the agent, and
// the firmware reconnects by itself. While it is gone, no DbwCommand arrives,
// so AUTONOMOUS falls to ESTOP after 500 ms (safety.md row 2).

#pragma once

#include "imu.h"

namespace ros_link {

void Begin();
void Spin();  // every main-loop pass
void OnImu(const imu::Sample &s);

}  // namespace ros_link
