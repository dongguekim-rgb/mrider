#include <gtest/gtest.h>
#include "../src/free_evidence.hpp"

using mrider_navigation_sim::cell_observed_free;
using mrider_navigation_sim::fresh_evidence;
using mrider_navigation_sim::valid_evidence;

nav_msgs::msg::OccupancyGrid grid()
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = 10; g.info.height = 10; g.info.resolution = 0.1;
  g.info.origin.orientation.w = 1.0; g.data.assign(100, 0);
  return g;
}

TEST(FreeEvidence, RequiresFreshSourceTime)
{
  EXPECT_TRUE(fresh_evidence(10.0, 10.2));
  EXPECT_FALSE(fresh_evidence(10.0, 10.36));
  EXPECT_FALSE(fresh_evidence(10.2, 10.0));
  EXPECT_FALSE(fresh_evidence(0.0, 0.1));
}

TEST(FreeEvidence, RejectsMalformedGeometry)
{
  auto g = grid(); EXPECT_TRUE(valid_evidence(g));
  g.data.pop_back(); EXPECT_FALSE(valid_evidence(g));
  g = grid(); g.info.origin.orientation.z = 0.1; EXPECT_FALSE(valid_evidence(g));
  g = grid(); g.info.resolution = 0.0; EXPECT_FALSE(valid_evidence(g));
}

TEST(FreeEvidence, UnknownOrOccupiedCornerPreventsClearingWholeCell)
{
  auto g = grid();
  EXPECT_TRUE(cell_observed_free(g, 0.5, 0.5, 0.1, 0, 0, 0));
  g.data[4 * 10 + 4] = -1;
  EXPECT_FALSE(cell_observed_free(g, 0.5, 0.5, 0.1, 0, 0, 0));
  g.data[4 * 10 + 4] = 100;
  EXPECT_FALSE(cell_observed_free(g, 0.5, 0.5, 0.1, 0, 0, 0));
}

TEST(FreeEvidence, MapEdgeIsNotObservedFree)
{
  const auto g = grid();
  EXPECT_FALSE(cell_observed_free(g, 0.01, 0.5, 0.1, 0, 0, 0));
  EXPECT_FALSE(cell_observed_free(g, 1.01, 0.5, 0.1, 0, 0, 0));
}

TEST(FreeEvidence, UsesFrameTransformAndMapOrigin)
{
  auto g = grid(); g.info.origin.position.x = 2.0; g.info.origin.position.y = 3.0;
  EXPECT_FALSE(cell_observed_free(g, 0.5, 0.5, 0.05, 0, 0, 0));
  EXPECT_TRUE(cell_observed_free(g, 0.5, 0.5, 0.05, 3.0, 3.0, 1.5707963267948966));
}
