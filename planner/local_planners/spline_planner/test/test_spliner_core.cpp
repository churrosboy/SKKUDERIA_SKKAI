// Unit tests + golden replay for the C++ port of spline_planner.py.
// Golden files come from test/gen_spliner_golden.py (drives the ORIGINAL python node).

#include <gtest/gtest.h>

#include <spline_planner/cubic_spline.hpp>
#include <spline_planner/spliner_core.hpp>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace
{

using spline_planner::CubicSpline;
using spline_planner::Spliner;
using spline_planner::SplinerParams;

TEST(CubicSplineTest, InterpolatesKnotsAndMatchesScipyNotAKnot)
{
  // scipy.interpolate.CubicSpline([0,1,2.5,4,5],[0,1,0,-1,0.5])(1.7) == 0.76228
  const std::vector<double> x{0.0, 1.0, 2.5, 4.0, 5.0};
  const std::vector<double> y{0.0, 1.0, 0.0, -1.0, 0.5};
  const CubicSpline spline(x, y);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(spline(x[i]), y[i], 1e-12);
  }
  EXPECT_NEAR(spline(1.7), 0.7622800000000001, 1e-9);
  EXPECT_NEAR(spline.derivative(1.7), -0.7107166666666667, 1e-9);
}

TEST(CubicSplineTest, FourPointsIsSingleCubic)
{
  const std::vector<double> x{0.0, 1.0, 2.0, 3.0};
  std::vector<double> y;
  for (const double v : x) {
    y.push_back(1.0 + 2.0 * v - 0.5 * v * v + 0.25 * v * v * v);
  }
  const CubicSpline spline(x, y);
  EXPECT_NEAR(spline(1.5), 1.0 + 3.0 - 0.5 * 2.25 + 0.25 * 3.375, 1e-12);
  EXPECT_NEAR(spline(4.0), 1.0 + 8.0 - 8.0 + 16.0, 1e-12);  // extrapolation
}

std::vector<f110_msgs::msg::Wpnt> straightTrack()
{
  std::vector<f110_msgs::msg::Wpnt> wpnts;
  for (int i = 0; i < 400; ++i) {
    f110_msgs::msg::Wpnt w;
    w.id = i;
    w.s_m = 0.1 * i;
    w.x_m = 0.1 * i;
    w.y_m = 0.0;
    w.psi_rad = 0.0;
    w.kappa_radpm = 0.0;
    w.d_left = 1.2;
    w.d_right = 1.2;
    w.vx_mps = 3.0;
    wpnts.push_back(w);
  }
  return wpnts;
}

f110_msgs::msg::Obstacle obstacle(double s, double d, double size)
{
  f110_msgs::msg::Obstacle o;
  o.s_center = s;
  o.s_start = s - size / 2.0;
  o.s_end = s + size / 2.0;
  o.d_center = d;
  o.d_left = d + size / 2.0;
  o.d_right = d - size / 2.0;
  o.size = size;
  o.is_static = true;
  o.is_visible = true;
  return o;
}

TEST(SplinerCore, EvadesSingleObstacleOnTheFreeSide)
{
  Spliner spliner(SplinerParams{});
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto out = spliner.step(false, false);
  ASSERT_TRUE(out.evaluated);
  EXPECT_FALSE(out.danger);
  EXPECT_EQ(out.ot_side, "right");  // obstacle sits left of the raceline
  EXPECT_TRUE(out.side_switch);     // first decision
  ASSERT_GT(out.wpnts.size(), 10U);
  double min_d = 0.0;
  for (const auto & w : out.wpnts) {
    min_d = std::min(min_d, w.d_m);
  }
  EXPECT_NEAR(min_d, 0.15 - 0.4, 1e-9);  // d_right - evasion_dist
  EXPECT_EQ(spliner.lastOtSide(), "right");
  // second cycle: same side -> no switch
  EXPECT_FALSE(spliner.step(false, false).side_switch);
}

TEST(SplinerCore, NoObstaclesDeletesMarkers)
{
  Spliner spliner(SplinerParams{});
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  const auto out = spliner.step(false, false);
  EXPECT_FALSE(out.evaluated);
  EXPECT_TRUE(out.delete_markers);
  EXPECT_TRUE(out.wpnts.empty());
}

TEST(SplinerCore, OffRacelineBlocksSideSwitch)
{
  Spliner spliner(SplinerParams{});
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.5, 3.0);  // |d| > 0.25 and no previous side
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto out = spliner.step(false, false);
  ASSERT_TRUE(out.evaluated);
  EXPECT_TRUE(out.danger);
  EXPECT_TRUE(out.wpnts.empty());
  EXPECT_TRUE(out.side_switch);
}

// car already alongside obstacle A (s_start behind, s_center ahead) must not
// drag a far obstacle B into the group
TEST(SplinerCore, InsideObstacleSpanDoesNotGroupFarObstacle)
{
  std::vector<std::string> logs;
  const auto logger = [&](spline_planner::LogLevel, const std::string & m) {logs.push_back(m);};
  auto grouped = [&]() {
      for (const auto & m : logs) {
        if (m.rfind("Grouped", 0) == 0) {
          return true;
        }
      }
      return false;
    };

  Spliner spliner(SplinerParams{}, logger);
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(8.05, 0.0, 3.0);  // inside A's span [8.0, 8.3]
  spliner.setObstacles({obstacle(8.15, 0.3, 0.3), obstacle(14.0, 0.3, 0.3)});  // 5.7 m gap
  const auto out = spliner.step(false, false);
  ASSERT_TRUE(out.evaluated);
  EXPECT_FALSE(grouped()) << "B (5.7 m ahead) was merged with A";
  EXPECT_FALSE(out.danger);
  ASSERT_FALSE(out.wpnts.empty());
  EXPECT_LT(out.wpnts.back().s_m, 13.0);  // evasion spans A only, not out to B

  // genuine chain (2.2 m gap < obs_group_gap_m 3.0) must still be merged
  logs.clear();
  spliner.setObstacles({obstacle(8.15, 0.3, 0.3), obstacle(10.5, 0.3, 0.3)});
  spliner.step(false, false);
  EXPECT_TRUE(grouped());
}

TEST(SplinerGolden, MatchesPythonNode)
{
  const std::string dir = SPLINER_GOLDEN_DIR;
  std::ifstream scenario(dir + "/spliner_scenario.txt");
  std::ifstream expected(dir + "/spliner_expected.txt");
  ASSERT_TRUE(scenario.is_open()) << "run test/gen_spliner_golden.py";
  ASSERT_TRUE(expected.is_open()) << "run test/gen_spliner_golden.py";

  std::string tag;
  std::size_t n = 0;
  scenario >> tag >> n;
  ASSERT_EQ(tag, "TRACK");
  std::vector<f110_msgs::msg::Wpnt> path(n), scaled(n);
  for (std::size_t i = 0; i < n; ++i) {
    auto & w = path[i];
    double vx_scaled = 0.0;
    scenario >> w.id >> w.s_m >> w.x_m >> w.y_m >> w.psi_rad >> w.kappa_radpm >> w.d_left >>
      w.d_right >> w.vx_mps >> vx_scaled;
    scaled[i] = w;
    scaled[i].vx_mps = vx_scaled;
  }
  Spliner spliner(SplinerParams{});
  spliner.setGlobalPath(path);
  spliner.setScaledPath(scaled);

  int frames = 0;
  while (scenario >> tag && tag != "END") {
    ASSERT_EQ(tag, "FRAME");
    int frame = 0;
    scenario >> frame;
    double s, d, vs;
    scenario >> tag >> s >> d >> vs;
    ASSERT_EQ(tag, "CAR");
    spliner.setState(s, d, vs);
    std::size_t n_obs = 0;
    scenario >> tag >> n_obs;
    ASSERT_EQ(tag, "OBS");
    std::vector<f110_msgs::msg::Obstacle> obs(n_obs);
    for (auto & o : obs) {
      int is_static = 0, is_visible = 0;
      scenario >> o.id >> o.s_start >> o.s_end >> o.s_center >> o.d_left >> o.d_right >>
        o.d_center >> o.size >> o.vs >> o.vd >> is_static >> is_visible;
      o.is_static = is_static != 0;
      o.is_visible = is_visible != 0;
    }
    spliner.setObstacles(obs);
    const auto out = spliner.step(false, false);

    int e_frame = 0;
    expected >> tag >> e_frame;
    ASSERT_EQ(tag, "FRAME");
    ASSERT_EQ(e_frame, frame);
    std::string frame_id, ot_side, ot_line;
    int side_switch = 0;
    std::size_t n_wpnts = 0;
    expected >> tag >> frame_id >> ot_side >> ot_line >> side_switch >> n_wpnts;
    ASSERT_EQ(tag, "OT");
    const std::string where = " frame " + std::to_string(frame);
    EXPECT_EQ(out.evaluated, frame_id == "map") << where;
    EXPECT_EQ(out.danger ? "-" : (out.ot_side.empty() ? "-" : out.ot_side), ot_side) << where;
    EXPECT_EQ(out.danger ? "-" : (out.ot_line.empty() ? "-" : out.ot_line), ot_line) << where;
    EXPECT_EQ(out.side_switch ? 1 : 0, side_switch) << where;
    ASSERT_EQ(out.wpnts.size(), n_wpnts) << where;
    for (std::size_t i = 0; i < n_wpnts; ++i) {
      int id = 0;
      double es, ed, ev, ex, ey;
      expected >> id >> es >> ed >> ev >> ex >> ey;
      const auto & w = out.wpnts[i];
      EXPECT_EQ(w.id, id) << where << " idx " << i;
      EXPECT_NEAR(w.s_m, es, 1e-9) << where << " idx " << i;
      EXPECT_NEAR(w.d_m, ed, 1e-9) << where << " idx " << i;
      EXPECT_NEAR(w.vx_mps, ev, 1e-12) << where << " idx " << i;
      EXPECT_NEAR(w.x_m, ex, 1e-7) << where << " idx " << i;
      EXPECT_NEAR(w.y_m, ey, 1e-7) << where << " idx " << i;
    }
    std::string last;
    expected >> tag >> last;
    ASSERT_EQ(tag, "LAST");
    EXPECT_EQ(spliner.lastOtSide().empty() ? "-" : spliner.lastOtSide(), last) << where;
    ++frames;
    if (::testing::Test::HasFailure()) {
      FAIL() << "stopping at first mismatching frame " << frame;
    }
  }
  EXPECT_EQ(frames, 300);
  // cache_enable is OFF by default: the replay above must be byte-identical to python AND
  // the cache must have stayed untouched (no lookup, no store)
  EXPECT_EQ(spliner.cacheHits(), 0U);
  EXPECT_EQ(spliner.cacheMisses(), 0U);
  EXPECT_EQ(spliner.cacheSize(), 0U);
}

// ---- evasion cache ---------------------------------------------------------------------

SplinerParams cacheParams()
{
  SplinerParams p;
  p.cache_enable = true;
  return p;
}

bool sameWpnts(
  const std::vector<f110_msgs::msg::Wpnt> & a, const std::vector<f110_msgs::msg::Wpnt> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id || a[i].s_m != b[i].s_m || a[i].d_m != b[i].d_m ||
      a[i].x_m != b[i].x_m || a[i].y_m != b[i].y_m || a[i].vx_mps != b[i].vx_mps)
    {
      return false;
    }
  }
  return true;
}

TEST(SplinerCache, SecondTickIsHitAndIdentical)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto first = spliner.step(true, false);
  ASSERT_TRUE(first.evaluated);
  EXPECT_FALSE(first.danger);
  EXPECT_EQ(spliner.cacheMisses(), 1U);
  EXPECT_EQ(spliner.cacheHits(), 0U);
  EXPECT_EQ(spliner.cacheSize(), 1U);

  // the car moved and the tracker mean crept by 1 cm: still the same obstacle
  spliner.setState(3.0, 0.0, 3.2);
  spliner.setObstacles({obstacle(8.01, 0.305, 0.3)});
  const auto second = spliner.step(true, false);
  EXPECT_EQ(spliner.cacheHits(), 1U);
  EXPECT_EQ(spliner.cacheSize(), 1U);
  EXPECT_TRUE(sameWpnts(first.wpnts, second.wpnts));
  EXPECT_EQ(second.ot_side, first.ot_side);
  EXPECT_EQ(second.ot_line, first.ot_line);
  EXPECT_FALSE(second.side_switch);  // hysteresis still tracked on a hit
  EXPECT_EQ(second.markers.size(), first.markers.size());
  EXPECT_TRUE(second.has_considered);
  EXPECT_DOUBLE_EQ(second.considered_x, first.considered_x);

  // same inputs with the cache off must give the same wpnts as the cached copy
  Spliner plain(SplinerParams{});
  plain.setGlobalPath(straightTrack());
  plain.setScaledPath(straightTrack());
  plain.setState(2.0, 0.0, 3.0);
  plain.setObstacles({obstacle(8.0, 0.3, 0.3)});
  EXPECT_TRUE(sameWpnts(plain.step(true, false).wpnts, first.wpnts));
}

TEST(SplinerCache, MovedObstacleMissesAndOverwritesSlot)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  spliner.step(false, false);
  // beyond cache_pos_tol_m (0.1) -> miss, recompute
  spliner.setObstacles({obstacle(8.5, 0.3, 0.3)});
  const auto moved = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheMisses(), 2U);
  EXPECT_EQ(spliner.cacheHits(), 0U);
  EXPECT_EQ(spliner.cacheSize(), 2U);  // a different position is a new slot
  // the old position is still remembered (lap-to-lap reuse)
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 1U);
  EXPECT_EQ(spliner.cacheSize(), 2U);
  ASSERT_FALSE(moved.wpnts.empty());
}

TEST(SplinerCache, SpeedChangeBeyondTolMissesAndOverwrites)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 1.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto slow = spliner.step(false, false);
  spliner.setState(2.0, 0.0, 1.5);  // within 1.0 m/s -> hit
  EXPECT_TRUE(sameWpnts(spliner.step(false, false).wpnts, slow.wpnts));
  EXPECT_EQ(spliner.cacheHits(), 1U);
  spliner.setState(2.0, 0.0, 2.5);  // 1.5 m/s away -> miss, same slot overwritten
  const auto fast = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheMisses(), 2U);
  EXPECT_EQ(spliner.cacheSize(), 1U);
  EXPECT_FALSE(sameWpnts(fast.wpnts, slow.wpnts));  // knots scale with speed
  spliner.setState(2.0, 0.0, 2.6);
  EXPECT_TRUE(sameWpnts(spliner.step(false, false).wpnts, fast.wpnts));
  EXPECT_EQ(spliner.cacheHits(), 2U);
}

TEST(SplinerCache, PerTickChecksStillRunOnHit)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto first = spliner.step(false, false);
  ASSERT_FALSE(first.danger);
  ASSERT_EQ(first.ot_side, "right");

  // a second obstacle evaded on the LEFT makes last_ot_side = left (cached as well)
  spliner.setObstacles({obstacle(10.0, -0.3, 0.3)});  // inside the 10 m lookahead
  ASSERT_EQ(spliner.step(false, false).ot_side, "left");
  ASSERT_EQ(spliner.cacheSize(), 2U);

  // side gate (|cur_d| > 0.25 and side switch): the cached right evasion is a switch from
  // left while the car sits 0.5 m off the raceline -> danger, even though it is a hit
  spliner.setState(2.0, 0.5, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto gated = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 1U);
  EXPECT_TRUE(gated.danger);
  EXPECT_TRUE(gated.wpnts.empty());

  // other obstacle sitting on the cached evasion line (s 11.5 is beyond the 3 m group gap,
  // inside the post-apex tail) -> danger, entry untouched
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3), obstacle(11.5, -0.25, 0.3)});
  const auto blocked = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 2U);
  EXPECT_TRUE(blocked.danger);
  EXPECT_EQ(spliner.cacheSize(), 2U);

  // blocker gone -> the cached evasion is valid again, unchanged
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  const auto again = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 3U);
  EXPECT_FALSE(again.danger);
  EXPECT_TRUE(sameWpnts(again.wpnts, first.wpnts));
}

TEST(SplinerCache, DynamicObstacleBypassesCache)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  auto moving = obstacle(8.0, 0.3, 0.3);
  moving.is_static = false;
  moving.vs = 1.0;
  spliner.setObstacles({moving});
  spliner.step(false, false);
  spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 0U);
  EXPECT_EQ(spliner.cacheMisses(), 0U);
  EXPECT_EQ(spliner.cacheSize(), 0U);
}

TEST(SplinerCache, BoundVetoIsCachedAsDanger)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  // obstacle 1.0 m wide on a 1.2 m half-width track: no side has evasion_dist + mindist
  spliner.setObstacles({obstacle(8.0, 0.0, 2.0)});
  const auto first = spliner.step(false, false);
  ASSERT_TRUE(first.evaluated);
  EXPECT_TRUE(first.danger);
  EXPECT_EQ(spliner.cacheSize(), 1U);
  const auto second = spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 1U);
  EXPECT_TRUE(second.danger);
  EXPECT_TRUE(second.wpnts.empty());
}

TEST(SplinerCache, ParamAndScaledPathChangesClearCache)
{
  Spliner spliner(cacheParams());
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});
  spliner.step(false, false);
  ASSERT_EQ(spliner.cacheSize(), 1U);

  // identical republish (sector_tuner every 0.5 s) must NOT clear
  spliner.setScaledPath(straightTrack());
  EXPECT_EQ(spliner.cacheSize(), 1U);
  // real content change (sector speed scaling) clears
  auto rescaled = straightTrack();
  rescaled[100].vx_mps = 2.5;
  spliner.setScaledPath(rescaled);
  EXPECT_EQ(spliner.cacheSize(), 0U);
  spliner.step(false, false);
  ASSERT_EQ(spliner.cacheSize(), 1U);

  // the node calls clearCache() from its parameter callback
  spliner.params().evasion_dist = 0.5;
  spliner.clearCache();
  EXPECT_EQ(spliner.cacheSize(), 0U);
}

TEST(SplinerCache, LruEvictionHonoursMaxEntries)
{
  SplinerParams p = cacheParams();
  p.cache_max_entries = 2;
  Spliner spliner(p);
  spliner.setGlobalPath(straightTrack());
  spliner.setScaledPath(straightTrack());
  spliner.setState(2.0, 0.0, 3.0);
  for (const double s : {8.0, 9.0, 10.0}) {
    spliner.setObstacles({obstacle(s, 0.3, 0.3)});
    spliner.step(false, false);
  }
  EXPECT_EQ(spliner.cacheSize(), 2U);
  spliner.setObstacles({obstacle(8.0, 0.3, 0.3)});  // evicted (least recently used)
  spliner.step(false, false);
  EXPECT_EQ(spliner.cacheMisses(), 4U);
  spliner.setObstacles({obstacle(10.0, 0.3, 0.3)});  // still there
  spliner.step(false, false);
  EXPECT_EQ(spliner.cacheHits(), 1U);
}

// Replays the golden scenario with the cache ON and compares against the cache-OFF run:
// side decisions identical every frame, geometry within the configured tolerances.
TEST(SplinerCache, GoldenReplayWithCacheStaysClose)
{
  const std::string dir = SPLINER_GOLDEN_DIR;
  std::ifstream scenario(dir + "/spliner_scenario.txt");
  ASSERT_TRUE(scenario.is_open()) << "run test/gen_spliner_golden.py";
  std::string tag;
  std::size_t n = 0;
  scenario >> tag >> n;
  ASSERT_EQ(tag, "TRACK");
  std::vector<f110_msgs::msg::Wpnt> path(n), scaled(n);
  for (std::size_t i = 0; i < n; ++i) {
    auto & w = path[i];
    double vx_scaled = 0.0;
    scenario >> w.id >> w.s_m >> w.x_m >> w.y_m >> w.psi_rad >> w.kappa_radpm >> w.d_left >>
      w.d_right >> w.vx_mps >> vx_scaled;
    scaled[i] = w;
    scaled[i].vx_mps = vx_scaled;
  }
  Spliner plain(SplinerParams{});
  Spliner cached(cacheParams());
  for (Spliner * s : {&plain, &cached}) {
    s->setGlobalPath(path);
    s->setScaledPath(scaled);
  }

  int frames = 0;
  while (scenario >> tag && tag != "END") {
    ASSERT_EQ(tag, "FRAME");
    int frame = 0;
    scenario >> frame;
    double s, d, vs;
    scenario >> tag >> s >> d >> vs;
    ASSERT_EQ(tag, "CAR");
    std::size_t n_obs = 0;
    scenario >> tag >> n_obs;
    ASSERT_EQ(tag, "OBS");
    std::vector<f110_msgs::msg::Obstacle> obs(n_obs);
    for (auto & o : obs) {
      int is_static = 0, is_visible = 0;
      scenario >> o.id >> o.s_start >> o.s_end >> o.s_center >> o.d_left >> o.d_right >>
        o.d_center >> o.size >> o.vs >> o.vd >> is_static >> is_visible;
      o.is_static = is_static != 0;
      o.is_visible = is_visible != 0;
    }
    plain.setState(s, d, vs);
    plain.setObstacles(obs);
    cached.setState(s, d, vs);
    cached.setObstacles(obs);
    const auto a = plain.step(false, false);
    const auto b = cached.step(false, false);
    const std::string where = " frame " + std::to_string(frame);
    EXPECT_EQ(a.evaluated, b.evaluated) << where;
    EXPECT_EQ(a.danger, b.danger) << where;
    EXPECT_EQ(a.ot_side, b.ot_side) << where;
    EXPECT_EQ(a.ot_line, b.ot_line) << where;
    EXPECT_EQ(a.side_switch, b.side_switch) << where;
    EXPECT_EQ(a.wpnts.empty(), b.wpnts.empty()) << where;
    if (!a.wpnts.empty() && !b.wpnts.empty()) {
      // geometry drift bound: the cached evasion was built for an obstacle at most
      // cache_pos_tol_m away and a speed at most cache_vs_tol_mps away
      double apex_a = 0.0, apex_b = 0.0;
      for (const auto & w : a.wpnts) {
        apex_a = std::abs(w.d_m) > std::abs(apex_a) ? w.d_m : apex_a;
      }
      for (const auto & w : b.wpnts) {
        apex_b = std::abs(w.d_m) > std::abs(apex_b) ? w.d_m : apex_b;
      }
      EXPECT_NEAR(apex_a, apex_b, 0.15) << where;
      EXPECT_NEAR(a.wpnts.front().s_m, b.wpnts.front().s_m, 1.5) << where;  // knot scale
    }
    ++frames;
    if (::testing::Test::HasFailure()) {
      FAIL() << "stopping at first mismatching frame " << frame;
    }
  }
  EXPECT_EQ(frames, 300);
  EXPECT_GT(cached.cacheHits(), 0U);
  EXPECT_EQ(plain.cacheSize(), 0U);
}

}  // namespace
