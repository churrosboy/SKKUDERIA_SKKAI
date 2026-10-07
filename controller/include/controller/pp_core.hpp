#ifndef CONTROLLER__PP_CORE_HPP_
#define CONTROLLER__PP_CORE_HPP_

// C++ port of controller/pp.py (PP_Controller): pure logic, no ROS dependencies,
// unit-tested against golden vectors from the python implementation (test/generate_pp_golden.py).

#include <optional>
#include <utility>
#include <vector>

namespace controller
{

// Mirrors the PP_Controller ctor args + the attributes controller_manager.py injects per cycle.
struct PpParams
{
  double t_clip_min{0.0};
  double t_clip_max{0.0};
  double m_l1{0.0};
  double q_l1{0.0};
  double speed_lookahead{0.0};
  double lat_err_coeff{0.0};
  double acc_scaler_for_steer{0.0};
  double dec_scaler_for_steer{0.0};
  double start_scale_speed{0.0};
  double end_scale_speed{0.0};
  double downscale_factor{0.0};
  double speed_lookahead_for_steer{0.0};

  double trailing_gap{0.0};
  double trailing_p_gain{0.0};
  double trailing_i_gain{0.0};
  double trailing_d_gain{0.0};
  double blind_trailing_speed{0.0};
  double trailing_vel_gain{0.0};
  double trailing_min_speed{0.0};
  bool trailing_creep_always{false};
  double trailing_stop_gap{0.3};
  double trailing_nose_offset{0.45};

  // injected per cycle by controller_manager.py (pp_cycle :753-757); plain params here
  double recovery_l1_gain{0.0};
  double recovery_t_clip_max{6.0};
  bool l1_curv_cap_enable{false};
  double l1_curv_cap_max_heading{1.57};

  double loop_rate{20.0};  // [Hz] dt = 1/loop_rate for the trailing integrator
  double wheelbase{0.0};
};

// One row of waypoint_array_in_map (local_waypoint_cb :661-675):
// [x, y, v, norm_trackbound, s, kappa, psi, ax]
struct LocalWaypoint
{
  double x{0.0};
  double y{0.0};
  double v{0.0};
  double norm_tb{0.0};
  double s{0.0};
  double kappa{0.0};
  double psi{0.0};
  double ax{0.0};
};

// self.opponent list [s_center, d_center, vs, is_static, is_visible, size]
struct Opponent
{
  double s_center{0.0};
  double d_center{0.0};
  double vs{0.0};
  bool is_static{false};
  bool is_visible{false};
  double size{0.0};
};

struct PpInput
{
  bool is_trailing{false};   // state == "StateType.TRAILING"
  bool is_recovery{false};   // state == "StateType.RECOVERY"
  double x{0.0};
  double y{0.0};
  double yaw{0.0};           // position_in_map[0, 0..2]
  double s{0.0};
  double d{0.0};
  double vs{0.0};
  double vd{0.0};            // position_in_map_frenet
  double speed_now{0.0};
  double acc_mean{0.0};      // np.mean(acc_now) -- pp.py only ever uses the mean
  double track_length{0.0};
  std::optional<Opponent> opponent;
  const std::vector<LocalWaypoint> * waypoints{nullptr};  // non-owning
};

struct PpOutput
{
  double speed{0.0};
  double acceleration{0.0};  // always 0 in pp.py, kept for the 7-tuple contract
  double jerk{0.0};          // always 0
  double steering_angle{0.0};
  double l1_x{0.0};
  double l1_y{0.0};
  double l1_distance{0.0};
  int idx_nearest_waypoint{0};
  bool trailing_ran{false};  // true iff trailing_controller() ran this cycle
  // trailing telemetry for /trailing/gap_data (values from the last cycle in
  // which trailing ran; matches reading the python object's attributes)
  double gap{0.0};
  double gap_should{0.0};
  double gap_error{0.0};
  double v_diff{0.0};
  double i_gap{0.0};
  double trailing_command{2.0};
};

class PpCore
{
public:
  explicit PpCore(const PpParams & params);

  // Live tuning: swap params, keep internal state (integrator, steer slew, ...).
  void setParams(const PpParams & params);

  // controller_manager resets ctrl.i_gap = 0 when the SM trailing target id
  // switches (_apply_sm_trailing_target :602-608).
  void resetTrailingIntegrator();

  // Port of PP_Controller.main_loop (pp.py :98-139).
  PpOutput mainLoop(const PpInput & in);

  // numpy/python semantics helpers (public for direct unit testing)
  // np.clip == min(max(v, lo), hi): hi wins when lo > hi; std::clamp would be UB -- do not substitute.
  static double npClip(double v, double lo, double hi);
  // python % is non-negative for positive modulus; std::fmod is not.
  static double pyMod(double a, double b);

private:
  double calcSteeringAngle(
    const PpInput & in, double l1_x, double l1_y, double l1_distance,
    double lat_e_norm);
  std::pair<double, double> calcL1Point(
    const PpInput & in, double lateral_error, double * l1_distance_out);
  double calcSpeedCommand(const PpInput & in, double lat_e_norm);
  double trailingController(const PpInput & in, double global_speed);
  double speedAdjustLatErr(double global_speed, double lat_e_norm) const;
  static int nearestWaypoint(
    double px, double py, const std::vector<LocalWaypoint> & waypoints);
  std::pair<double, double> waypointAtDistanceBeforeCar(
    double distance, const std::vector<LocalWaypoint> & waypoints,
    int idx_waypoint_behind_car) const;

  PpParams p_;

  // Persistent controller state (python object attributes)
  double curr_steering_angle_{0.0};   // steer slew base, +-0.4 rad per cycle
  int idx_nearest_waypoint_{0};
  double i_gap_{0.0};                 // trailing PID integrator, clip +-10
  double curvature_waypoints_{0.0};   // retained when <3 waypoints remain past idx
  // trailing telemetry (python inits trailing_command = 2)
  double gap_{0.0};
  double gap_should_{0.0};
  double gap_error_{0.0};
  double v_diff_{0.0};
  double trailing_command_{2.0};
};

}  // namespace controller

#endif  // CONTROLLER__PP_CORE_HPP_
