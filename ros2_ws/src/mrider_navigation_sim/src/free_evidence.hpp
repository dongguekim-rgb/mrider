#pragma once

#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace mrider_navigation_sim
{
inline bool fresh_evidence(double stamp, double now)
{
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0.0 &&
         now >= stamp && now - stamp <= 0.35;
}

inline bool valid_evidence(const nav_msgs::msg::OccupancyGrid & grid)
{
  const auto & i = grid.info;
  const auto & p = i.origin.position;
  const auto & q = i.origin.orientation;
  return i.width > 0 && i.height > 0 && std::isfinite(i.resolution) &&
         i.resolution > 0.0 && i.resolution <= 1.0 &&
         grid.data.size() == static_cast<size_t>(i.width) * i.height &&
         std::isfinite(p.x) && std::isfinite(p.y) &&
         q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 1.0;
}

// All corners and centre of a costmap cell must be observed free. tx/ty/yaw
// transform the costmap's frame to the evidence frame at its source time.
inline bool cell_observed_free(
  const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy,
  double cell_size, double tx, double ty, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double h = cell_size / 2.0;
  const double offsets[5][2] = {{0, 0}, {-h, -h}, {-h, h}, {h, -h}, {h, h}};
  for (const auto & offset : offsets) {
    const double x = tx + c * (wx + offset[0]) - s * (wy + offset[1]);
    const double y = ty + s * (wx + offset[0]) + c * (wy + offset[1]);
    const int col = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
    const int row = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
    if (col < 0 || row < 0 || col >= static_cast<int>(grid.info.width) ||
      row >= static_cast<int>(grid.info.height) ||
      grid.data[static_cast<size_t>(row) * grid.info.width + col] != 0)
    {
      return false;
    }
  }
  return true;
}
}  // namespace mrider_navigation_sim
