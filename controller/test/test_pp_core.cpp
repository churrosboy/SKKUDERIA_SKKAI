// Parity tests for the C++ port of PP_Controller (pp_core) against golden
// vectors generated from the python implementation by generate_pp_golden.py.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "controller/pp_core.hpp"

using controller::LocalWaypoint;
using controller::Opponent;
using controller::PpCore;
using controller::PpInput;
using controller::PpParams;

namespace
{

struct GoldenOpponent
{
  bool present;
  double s_center;
  double d_center;
  double vs;
  bool is_static;
  bool is_visible;
  double size;
};

// Field order must match the row emitter in generate_pp_golden.py (emit_inc).
struct GoldenCycle
{
  bool is_trailing;
  bool is_recovery;
  double x, y, yaw;
  double s, d, vs, vd;
  double speed_now;
  double acc_mean;
  double track_length;
  GoldenOpponent opponent;
  // expected outputs
  double e_speed;
  double e_steer;
  double e_l1x, e_l1y, e_l1d;
  int e_idx;
  bool e_trailing_ran;
  double e_gap, e_gap_should, e_gap_error, e_v_diff, e_i_gap, e_trailing_cmd;
};

struct GoldenScenario
{
  std::string name;
  PpParams params;
  std::vector<std::array<double, 8>> wpts;
  std::vector<GoldenCycle> cycles;
};

#include "data/pp_golden.inc"

constexpr double kTol = 1e-9;  // identical double math; libm differences << 1e-12

}  // namespace

TEST(PpCoreGolden, MatchesPythonImplementation)
{
  ASSERT_FALSE(kGoldenScenarios.empty());
  for (const auto & sc : kGoldenScenarios) {
    SCOPED_TRACE(sc.name);
    PpCore core(sc.params);
    std::vector<LocalWaypoint> wpts;
    wpts.reserve(sc.wpts.size());
    for (const auto & w : sc.wpts) {
      wpts.push_back({w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]});
    }
    int cycle_no = 0;
    for (const auto & c : sc.cycles) {
      SCOPED_TRACE("cycle " + std::to_string(cycle_no++));
      PpInput in;
      in.is_trailing = c.is_trailing;
      in.is_recovery = c.is_recovery;
      in.x = c.x;
      in.y = c.y;
      in.yaw = c.yaw;
      in.s = c.s;
      in.d = c.d;
      in.vs = c.vs;
      in.vd = c.vd;
      in.speed_now = c.speed_now;
      in.acc_mean = c.acc_mean;
      in.track_length = c.track_length;
      if (c.opponent.present) {
        in.opponent = Opponent{c.opponent.s_center, c.opponent.d_center,
          c.opponent.vs, c.opponent.is_static, c.opponent.is_visible,
          c.opponent.size};
      }
      in.waypoints = &wpts;

      const auto out = core.mainLoop(in);
      EXPECT_NEAR(out.speed, c.e_speed, kTol);
      EXPECT_NEAR(out.steering_angle, c.e_steer, kTol);
      EXPECT_NEAR(out.l1_x, c.e_l1x, kTol);
      EXPECT_NEAR(out.l1_y, c.e_l1y, kTol);
      EXPECT_NEAR(out.l1_distance, c.e_l1d, kTol);
      EXPECT_EQ(out.idx_nearest_waypoint, c.e_idx);
      EXPECT_EQ(out.trailing_ran, c.e_trailing_ran);
      EXPECT_NEAR(out.i_gap, c.e_i_gap, kTol);
      if (c.e_trailing_ran) {
        EXPECT_NEAR(out.gap, c.e_gap, kTol);
        EXPECT_NEAR(out.gap_should, c.e_gap_should, kTol);
        EXPECT_NEAR(out.gap_error, c.e_gap_error, kTol);
        EXPECT_NEAR(out.v_diff, c.e_v_diff, kTol);
        EXPECT_NEAR(out.trailing_command, c.e_trailing_cmd, kTol);
      }
    }
  }
}

TEST(PpCoreHelpers, PyModIsNonNegative)
{
  EXPECT_DOUBLE_EQ(PpCore::pyMod(-3.0, 10.0), 7.0);
  EXPECT_DOUBLE_EQ(PpCore::pyMod(3.0, 10.0), 3.0);
  EXPECT_DOUBLE_EQ(PpCore::pyMod(13.0, 10.0), 3.0);
  // trailing s-wrap case: opponent just past origin, ego near track end
  EXPECT_DOUBLE_EQ(PpCore::pyMod(1.0 - 99.0, 100.0), 2.0);
}

TEST(PpCoreHelpers, NpClipHighWinsWhenBoundsCross)
{
  // np.clip(v, lo, hi) == min(max(v, lo), hi): with lo > hi the result is hi.
  // Reachable via lower_bound = sqrt(2)*lateral_error > t_clip_max off-track.
  EXPECT_DOUBLE_EQ(PpCore::npClip(3.0, 5.66, 5.0), 5.0);
  EXPECT_DOUBLE_EQ(PpCore::npClip(0.2, 0.5, 5.0), 0.5);
  EXPECT_DOUBLE_EQ(PpCore::npClip(7.0, 0.5, 5.0), 5.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
