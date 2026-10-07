// Unit tests + golden replay for the C++ port of perception/tracking.py
// (golden files are produced by test/gen_tracking_golden.py from the python node).

#include <gtest/gtest.h>

#include <perception/tracking_core.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

using perception::ScanData;
using perception::Tracker;
using perception::TrackingParams;

std::vector<f110_msgs::msg::Wpnt> straightTrack(double length, double spacing)
{
  std::vector<f110_msgs::msg::Wpnt> wpnts;
  for (double s = 0.0; s <= length + 1e-9; s += spacing) {
    f110_msgs::msg::Wpnt w;
    w.s_m = s;
    w.x_m = s;
    w.y_m = 0.0;
    w.psi_rad = 0.0;
    w.vx_mps = 3.0;
    wpnts.push_back(w);
  }
  return wpnts;
}

f110_msgs::msg::Obstacle meas(double s, double d, double size = 0.3)
{
  f110_msgs::msg::Obstacle o;
  o.s_center = s;
  o.d_center = d;
  o.size = size;
  return o;
}

TEST(TrackingCore, PyModAndNormalizeS)
{
  EXPECT_DOUBLE_EQ(perception::pyMod(-1.0, 40.0), 39.0);
  EXPECT_DOUBLE_EQ(perception::pyMod(41.0, 40.0), 1.0);
  EXPECT_DOUBLE_EQ(perception::pyMod(0.0, 40.0), 0.0);
  EXPECT_DOUBLE_EQ(perception::normalizeS(39.0, 40.0), -1.0);
  EXPECT_DOUBLE_EQ(perception::normalizeS(-39.0, 40.0), 1.0);
  EXPECT_DOUBLE_EQ(perception::normalizeS(20.0, 40.0), 20.0);
  EXPECT_DOUBLE_EQ(perception::normalizeS(20.5, 40.0), -19.5);
}

TEST(TrackingCore, StaticObstacleIsClassifiedAndPublished)
{
  TrackingParams params;
  params.rate_hz = 20.0;
  Tracker tracker(params);
  ASSERT_TRUE(tracker.setGlobalPath(straightTrack(40.0, 0.1)));
  tracker.setCarS(1.0);
  tracker.setCarPose(1.0, 0.0, 0.0);

  for (int i = 0; i < 10; ++i) {
    tracker.setMeasurements({meas(5.0 + 0.001 * i, 0.2)});
    tracker.step();
  }
  ASSERT_EQ(tracker.trackedObstacles().size(), 1U);
  EXPECT_EQ(tracker.trackedObstacles().front().static_flag, perception::StaticFlag::Static);

  std::vector<f110_msgs::msg::Obstacle> est, raw;
  tracker.buildObstacleArrays(est, raw);
  ASSERT_EQ(est.size(), 1U);
  EXPECT_TRUE(raw.empty());
  EXPECT_TRUE(est.front().is_static);
  EXPECT_NEAR(est.front().s_center, 5.0045, 1e-3);
  EXPECT_NEAR(est.front().d_center, 0.2, 1e-9);
  EXPECT_NEAR(est.front().s_start, est.front().s_center - 0.15, 1e-12);
}

TEST(TrackingCore, MovingObstacleInitialisesOpponentKf)
{
  TrackingParams params;
  params.rate_hz = 20.0;
  Tracker tracker(params);
  ASSERT_TRUE(tracker.setGlobalPath(straightTrack(40.0, 0.1)));
  tracker.setCarS(0.0);
  tracker.setCarPose(0.0, 0.0, 0.0);

  for (int i = 0; i < 30; ++i) {
    tracker.setMeasurements({meas(3.0 + 0.1 * i, 0.0)});  // 2 m/s at 20 Hz
    tracker.step();
  }
  ASSERT_EQ(tracker.trackedObstacles().size(), 1U);
  EXPECT_EQ(tracker.trackedObstacles().front().static_flag, perception::StaticFlag::Dynamic);
  ASSERT_TRUE(tracker.opponent().is_initialised);
  EXPECT_NEAR(tracker.opponent().x[1], 2.0, 0.2);

  std::vector<f110_msgs::msg::Obstacle> est, raw;
  tracker.buildObstacleArrays(est, raw);
  ASSERT_EQ(raw.size(), 1U);  // the raw dynamic track goes to /perception/raw_obstacles
  ASSERT_EQ(est.size(), 1U);  // the KF opponent goes to /perception/obstacles
  EXPECT_FALSE(est.front().is_static);
  EXPECT_NEAR(est.front().vs, 2.0, 0.3);

  // lose the track: dynamic ttl runs out, the KF coasts on the target velocity
  for (int i = 0; i < 5; ++i) {
    tracker.setMeasurements({});
    tracker.step();
  }
  EXPECT_TRUE(tracker.trackedObstacles().empty());
  EXPECT_TRUE(tracker.opponent().is_initialised);
  EXPECT_TRUE(tracker.opponent().use_target_vel);
}

TEST(TrackingCore, FieldOfViewUsesScanSlice)
{
  TrackingParams params;
  Tracker tracker(params);
  ScanData scan;
  scan.angle_min = -M_PI / 2.0;
  scan.angle_max = M_PI / 2.0;
  scan.angle_increment = M_PI / 180.0;
  scan.ranges.assign(181, 9.0F);
  tracker.setScan(scan);
  // obstacle 3 m straight ahead: beams reach 9 m -> the spot is visible
  EXPECT_TRUE(tracker.checkInFieldOfView(3.0, 0.0, 1.0, 0.0));
  // occluded: beams around the bearing stop at 1 m
  for (int i = 86; i < 95; ++i) {
    scan.ranges[static_cast<std::size_t>(i)] = 1.0F;
  }
  tracker.setScan(scan);
  EXPECT_FALSE(tracker.checkInFieldOfView(3.0, 0.0, 1.0, 0.0));
  // behind the car: outside the scan angles
  EXPECT_FALSE(tracker.checkInFieldOfView(-3.0, 0.0, 1.0, 0.0));
  // no scan yet
  tracker.setScan(ScanData{});
  EXPECT_FALSE(tracker.checkInFieldOfView(3.0, 0.0, 1.0, 0.0));
}

// ---------------------------------------------------------------------------
// golden replay against the python node
// ---------------------------------------------------------------------------

struct ExpectedObstacle
{
  int id;
  double s_center, d_center, s_start, s_end, d_right, d_left, size, vs, vd;
  int is_static, is_visible;
};

struct ExpectedFrame
{
  int frame;
  std::vector<ExpectedObstacle> est;
  std::vector<ExpectedObstacle> raw;
  int opp_init;
  int opp_id;
  double opp_x[4];
  double opp_p00;
  int opp_ttl;
  int opp_use_target;
  int lap;
  int n_tracked;
};

ExpectedObstacle readObstacle(std::istream & in)
{
  ExpectedObstacle o;
  in >> o.id >> o.s_center >> o.d_center >> o.s_start >> o.s_end >> o.d_right >> o.d_left >>
    o.size >> o.vs >> o.vd >> o.is_static >> o.is_visible;
  return o;
}

void compareObstacles(
  const std::vector<f110_msgs::msg::Obstacle> & got,
  const std::vector<ExpectedObstacle> & expected, int frame, const char * which)
{
  ASSERT_EQ(got.size(), expected.size()) << which << " count differs at frame " << frame;
  for (std::size_t i = 0; i < got.size(); ++i) {
    const auto & g = got[i];
    const auto & e = expected[i];
    const std::string where = std::string(which) + " frame " + std::to_string(frame) +
      " idx " + std::to_string(i);
    EXPECT_EQ(g.id, e.id) << where;
    EXPECT_NEAR(g.s_center, e.s_center, 1e-6) << where;
    EXPECT_NEAR(g.d_center, e.d_center, 1e-6) << where;
    EXPECT_NEAR(g.s_start, e.s_start, 1e-6) << where;
    EXPECT_NEAR(g.s_end, e.s_end, 1e-6) << where;
    EXPECT_NEAR(g.d_right, e.d_right, 1e-6) << where;
    EXPECT_NEAR(g.d_left, e.d_left, 1e-6) << where;
    EXPECT_NEAR(g.size, e.size, 1e-9) << where;
    EXPECT_NEAR(g.vs, e.vs, 1e-5) << where;
    EXPECT_NEAR(g.vd, e.vd, 1e-5) << where;
    EXPECT_EQ(g.is_static ? 1 : 0, e.is_static) << where;
    EXPECT_EQ(g.is_visible ? 1 : 0, e.is_visible) << where;
  }
}

TEST(TrackingGolden, MatchesPythonNode)
{
  const std::string dir = TRACKING_GOLDEN_DIR;
  std::ifstream scenario(dir + "/tracking_scenario.txt");
  std::ifstream expected(dir + "/tracking_expected.txt");
  ASSERT_TRUE(scenario.is_open()) << "missing golden scenario (run test/gen_tracking_golden.py)";
  ASSERT_TRUE(expected.is_open()) << "missing golden expectations (run test/gen_tracking_golden.py)";

  std::string tag;
  scenario >> tag;
  ASSERT_EQ(tag, "PARAMS");
  TrackingParams params;
  scenario >> params.rate_hz;

  scenario >> tag;
  ASSERT_EQ(tag, "TRACK");
  std::size_t n_wpnts = 0;
  scenario >> n_wpnts;
  std::vector<f110_msgs::msg::Wpnt> wpnts(n_wpnts);
  for (auto & w : wpnts) {
    scenario >> w.s_m >> w.x_m >> w.y_m >> w.psi_rad >> w.vx_mps;
  }

  Tracker tracker(params);
  ASSERT_TRUE(tracker.setGlobalPath(wpnts));

  int frames_checked = 0;
  while (scenario >> tag) {
    if (tag == "END") {
      break;
    }
    ASSERT_EQ(tag, "FRAME");
    int frame = 0;
    scenario >> frame;

    scenario >> tag;
    ASSERT_EQ(tag, "CAR");
    double car_s, car_x, car_y, car_yaw;
    scenario >> car_s >> car_x >> car_y >> car_yaw;
    tracker.setCarS(car_s);
    tracker.setCarPose(car_x, car_y, car_yaw);

    scenario >> tag;
    ASSERT_EQ(tag, "SCAN");
    ScanData scan;
    std::size_t n_ranges = 0;
    scenario >> scan.angle_min >> scan.angle_max >> scan.angle_increment >> n_ranges;
    scan.ranges.resize(n_ranges);
    for (auto & r : scan.ranges) {
      scenario >> r;
    }
    tracker.setScan(std::move(scan));

    scenario >> tag;
    ASSERT_EQ(tag, "MEAS");
    std::size_t n_meas = 0;
    scenario >> n_meas;
    std::vector<f110_msgs::msg::Obstacle> obstacles(n_meas);
    for (auto & o : obstacles) {
      scenario >> o.s_center >> o.d_center >> o.size;
    }
    tracker.setMeasurements(std::move(obstacles));

    tracker.step();

    // expectations for this frame
    ExpectedFrame ef;
    expected >> tag >> ef.frame;
    ASSERT_EQ(tag, "FRAME");
    ASSERT_EQ(ef.frame, frame);
    std::size_t n = 0;
    expected >> tag >> n;
    ASSERT_EQ(tag, "EST");
    for (std::size_t i = 0; i < n; ++i) {
      ef.est.push_back(readObstacle(expected));
    }
    expected >> tag >> n;
    ASSERT_EQ(tag, "RAW");
    for (std::size_t i = 0; i < n; ++i) {
      ef.raw.push_back(readObstacle(expected));
    }
    expected >> tag >> ef.opp_init >> ef.opp_id >> ef.opp_x[0] >> ef.opp_x[1] >> ef.opp_x[2] >>
      ef.opp_x[3] >> ef.opp_p00 >> ef.opp_ttl >> ef.opp_use_target;
    ASSERT_EQ(tag, "OPP");
    expected >> tag >> ef.lap >> ef.n_tracked;
    ASSERT_EQ(tag, "STATE");

    std::vector<f110_msgs::msg::Obstacle> est, raw;
    tracker.buildObstacleArrays(est, raw);
    compareObstacles(est, ef.est, frame, "EST");
    compareObstacles(raw, ef.raw, frame, "RAW");

    const auto & opp = tracker.opponent();
    EXPECT_EQ(opp.is_initialised ? 1 : 0, ef.opp_init) << "frame " << frame;
    EXPECT_EQ(opp.use_target_vel ? 1 : 0, ef.opp_use_target) << "frame " << frame;
    if (ef.opp_init) {
      EXPECT_EQ(opp.id, ef.opp_id) << "frame " << frame;
      EXPECT_EQ(opp.ttl, ef.opp_ttl) << "frame " << frame;
      for (int k = 0; k < 4; ++k) {
        EXPECT_NEAR(opp.x[k], ef.opp_x[k], 1e-5) << "frame " << frame << " x" << k;
      }
      EXPECT_NEAR(opp.P(0, 0), ef.opp_p00, 1e-8) << "frame " << frame;
    }
    EXPECT_EQ(tracker.currentLap(), ef.lap) << "frame " << frame;
    EXPECT_EQ(static_cast<int>(tracker.trackedObstacles().size()), ef.n_tracked) <<
      "frame " << frame;
    ++frames_checked;
    if (::testing::Test::HasFailure()) {
      FAIL() << "stopping at first mismatching frame " << frame;
    }
  }
  EXPECT_GT(frames_checked, 100);
}

}  // namespace
