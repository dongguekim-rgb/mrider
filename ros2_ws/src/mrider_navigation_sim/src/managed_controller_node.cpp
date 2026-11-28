// Copyright (c) 2019 Intel Corporation
// Modifications: MRider team, 2026.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by law or agreed to in writing, software is distributed
// on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Humble ControllerServer::computeControl orchestration, with an interruptible
// costmap wait. The installed controller, progress and goal-checker plugins,
// parameters and FollowPath interface remain in use.
// Source: https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav2_controller/controller_server.hpp>

class ManagedController : public nav2_controller::ControllerServer
{
protected:
  // Expose a pointer to the protected callback slot, not an object cast.
  // This is compile-checked against Humble's header and leaves the original
  // action server/executor lifetime intact. Reconstructing an executor here
  // can race its just-started spin thread and stall lifecycle configuration.
  struct CallbackSlot : ActionServer
  {
    static auto member() {return &CallbackSlot::execute_callback_;}
  };

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = ControllerServer::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }
    // Configuration is inactive, so no execution callback can be running.
    action_server_.get()->*CallbackSlot::member() =
      std::bind(&ManagedController::control, this);
    return result;
  }

  void control()
  {
    std::lock_guard<std::mutex> lock(dynamic_params_lock_);
    try {
      const auto goal = action_server_->get_current_goal();
      if (!findControllerId(goal->controller_id, current_controller_) ||
        !findGoalCheckerId(goal->goal_checker_id, current_goal_checker_))
      {
        action_server_->terminate_current();
        return;
      }
      setPlannerPath(goal->path);
      progress_checker_->reset();
      last_valid_cmd_time_ = now();
      rclcpp::WallRate rate(controller_frequency_);
      using Steady = std::chrono::steady_clock;
      std::optional<Steady::time_point> stale_since;

      while (rclcpp::ok()) {
        if (!action_server_->is_server_active()) {
          return;
        }
        if (action_server_->is_cancel_requested()) {
          publishZeroVelocity();
          action_server_->terminate_all();
          return;
        }
        if (!costmap_ros_->isCurrent()) {
          publishZeroVelocity();
          if (!stale_since) {
            stale_since = Steady::now();
          }
          if (Steady::now() - *stale_since >= std::chrono::milliseconds(350)) {
            RCLCPP_ERROR(get_logger(), "Costmap remained stale; aborting FollowPath");
            action_server_->terminate_current();
            return;
          }
          // Unlike Humble's nested isCurrent() loop, the next iteration
          // rechecks cancellation/deactivation/shutdown as well as freshness.
          rate.sleep();
          continue;
        }
        stale_since.reset();
        updateGlobalPath();
        computeAndPublishVelocity();
        if (isGoalReached()) {
          if (publish_zero_velocity_) {
            publishZeroVelocity();
          }
          action_server_->succeeded_current();
          return;
        }
        rate.sleep();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      if (rclcpp::ok()) {
        publishZeroVelocity();
        action_server_->terminate_current();
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
