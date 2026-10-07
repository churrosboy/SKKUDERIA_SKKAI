#include <gtest/gtest.h>

#include <perception/detection_core.hpp>

#include <cmath>
#include <vector>

namespace
{

using perception::Cluster;
using perception::DetectionConfig;
using perception::DetectedObstacle;
using perception::ScanPoint;

TEST(DetectionCore, ClusteringSkipsOffTrackAndSmallClusters)
{
  DetectionConfig config;
  config.lambda_rad = 10.0 * M_PI / 180.0;
  config.min_points = 3U;

  const std::vector<ScanPoint> points{
    {0.00, 0.0, 1.0, true}, {0.02, 0.0, 1.0, true}, {0.04, 0.0, 1.0, true},
    {5.00, 0.0, 5.0, true},
    {1.00, 0.0, 1.0, true}, {1.02, 0.0, 1.0, true}};

  const auto clusters = perception::clusterScan(
    points, 0.01, config,
    [](double x, double) {return x < 2.0;});

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().size(), 3U);
}

TEST(DetectionCore, ClusteringReconnectsAnEarlierCluster)
{
  DetectionConfig config;
  config.lambda_rad = 10.0 * M_PI / 180.0;
  config.min_points = 1U;
  config.new_cluster_threshold_m = 0.4;

  const std::vector<ScanPoint> points{
    {0.0, 0.0, 1.0, true}, {10.0, 0.0, 1.0, true}, {0.1, 0.0, 1.0, true}};
  const auto clusters = perception::clusterScan(
    points, 0.01, config, [](double, double) {return true;});

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters.back().size(), 2U);
  EXPECT_DOUBLE_EQ(clusters.back().back().x, 0.1);
}

TEST(DetectionCore, LShapeFitProducesFiniteMinimumSizedObstacle)
{
  DetectionConfig config;
  config.min_obstacle_size_m = 0.2;
  const Cluster cluster{
    {1.0, 1.0, 1.0, true}, {1.1, 1.0, 1.0, true},
    {1.2, 1.0, 1.0, true}, {1.2, 1.1, 1.0, true},
    {1.2, 1.2, 1.0, true}};

  const auto obstacles = perception::fitLShapes({cluster}, {0.0, 0.0}, config);

  ASSERT_EQ(obstacles.size(), 1U);
  EXPECT_TRUE(std::isfinite(obstacles.front().center_x));
  EXPECT_TRUE(std::isfinite(obstacles.front().center_y));
  EXPECT_GE(obstacles.front().size, 0.2);
}

TEST(DetectionCore, FilteringRemovesOversizedObstaclesAndAssignsDenseIds)
{
  DetectionConfig config;
  config.max_obstacle_size_m = 1.0;
  const std::vector<DetectedObstacle> input{
    {-1, 0.0, 0.0, 0.4, 0.0}, {-1, 1.0, 0.0, 1.1, 0.0},
    {-1, 2.0, 0.0, 0.5, 0.0}};

  const auto filtered = perception::filterAndNumberObstacles(input, config);

  ASSERT_EQ(filtered.size(), 2U);
  EXPECT_EQ(filtered[0].id, 0);
  EXPECT_EQ(filtered[1].id, 1);
}

TEST(DetectionCore, PositiveWrappingHandlesTrackSeam)
{
  EXPECT_NEAR(perception::wrapPositive(-0.2, 10.0), 9.8, 1e-12);
  EXPECT_NEAR(perception::wrapPositive(10.2, 10.0), 0.2, 1e-12);
}

}  // namespace
