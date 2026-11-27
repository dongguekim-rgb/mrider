// Team-owned extension; base Nav2 marking, raytracing and freshness remain active.
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>

#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include "free_evidence.hpp"

namespace mrider_navigation_sim
{
class ObservedObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override
  {
    nav2_costmap_2d::ObstacleLayer::onInitialize();
    auto node = node_.lock();
    evidence_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/navigation/free_evidence", rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_ = msg;
      });
    status_ = node->create_publisher<std_msgs::msg::String>("/mrider_sim/costmap_evidence_status", 10);
    status_->on_activate();
  }

  void updateBounds(
    double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    applyEvidence(min_x, min_y, max_x, max_y);
    // Mark CURRENT returns after historical clearing. New obstacles win,
    // even when an older free observation covered the same cell.
    nav2_costmap_2d::ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
  }

private:
  void applyEvidence(double * min_x, double * min_y, double * max_x, double * max_y)
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr evidence;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      evidence = evidence_;
    }
    if (!evidence || !valid_evidence(*evidence)) {return;}
    const double stamp = rclcpp::Time(evidence->header.stamp).seconds();
    const double now = clock_->now().seconds();
    if (!fresh_evidence(stamp, now) || stamp <= applied_stamp_ ||
      evidence->header.frame_id != "map") {return;}
    double tx = 0.0, ty = 0.0, yaw = 0.0;
    if (global_frame_ != evidence->header.frame_id) {
      try {
        const auto tf = tf_->lookupTransform(
          evidence->header.frame_id, global_frame_, rclcpp::Time(evidence->header.stamp),
          rclcpp::Duration::from_seconds(0.0));
        const auto & q = tf.transform.rotation;
        tx = tf.transform.translation.x; ty = tf.transform.translation.y;
        yaw = 2.0 * std::atan2(q.z, q.w);
        if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(yaw) ||
          std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6) {return;}
      } catch (const tf2::TransformException &) {return;}
    }
    unsigned int cleared = 0;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
      for (unsigned int row = 0; row < getSizeInCellsY(); ++row) {
        for (unsigned int col = 0; col < getSizeInCellsX(); ++col) {
          if (getCost(col, row) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
          double wx, wy; mapToWorld(col, row, wx, wy);
          if (cell_observed_free(*evidence, wx, wy, getResolution(), tx, ty, yaw)) {
            setCost(col, row, nav2_costmap_2d::FREE_SPACE);
            touch(wx, wy, min_x, min_y, max_x, max_y); ++cleared;
          }
        }
      }
    }
    applied_stamp_ = stamp;
    std::ostringstream text;
    text << std::setprecision(17) << "{\"frame\":\"" << global_frame_
         << "\",\"stamp\":" << now << ",\"evidence_stamp\":" << stamp
         << ",\"cleared_cells\":" << cleared << "}";
    std_msgs::msg::String msg; msg.data = text.str(); status_->publish(msg);
  }

  std::mutex evidence_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr evidence_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr evidence_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
  double applied_stamp_{0.0};
};
}  // namespace mrider_navigation_sim

PLUGINLIB_EXPORT_CLASS(mrider_navigation_sim::ObservedObstacleLayer, nav2_costmap_2d::Layer)
