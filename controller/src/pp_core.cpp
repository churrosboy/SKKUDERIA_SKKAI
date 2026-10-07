// Literal C++ port of controller/pp.py (PP_Controller); evaluation ORDER matches python
// (calc_speed_command runs BEFORE calc_L1_point). Line references are into pp.py.

#include "controller/pp_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace controller
{

PpCore::PpCore(const PpParams & params)
: p_(params)
{
}

void PpCore::setParams(const PpParams & params)
{
  p_ = params;
}

void PpCore::resetTrailingIntegrator()
{
  i_gap_ = 0.0;
}

double PpCore::npClip(double v, double lo, double hi)
{
  // np.clip: min(max(v, lo), hi) -- hi wins when lo > hi (see header)
  return std::min(std::max(v, lo), hi);
}

double PpCore::pyMod(double a, double b)
{
  double r = std::fmod(a, b);
  return r < 0.0 ? r + b : r;
}

int PpCore::nearestWaypoint(
  double px, double py, const std::vector<LocalWaypoint> & waypoints)
{
  // pp.py :411-420 argmin of Euclidean distance (the python's abs()/copy are no-ops)
  int best_idx = 0;
  double best_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const double dx = px - waypoints[i].x;
    const double dy = py - waypoints[i].y;
    const double sq = dx * dx + dy * dy;
    if (sq < best_sq) {
      best_sq = sq;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

std::pair<double, double> PpCore::waypointAtDistanceBeforeCar(
  double distance, const std::vector<LocalWaypoint> & waypoints,
  int idx_waypoint_behind_car) const
{
  // pp.py :422-434 (the `distance is None` branch is unreachable from
  // calc_L1_point, which always passes a number)
  const double waypoints_distance = 0.1;
  const int d_index = static_cast<int>(distance / waypoints_distance + 0.5);
  const int idx = std::min(
    static_cast<int>(waypoints.size()) - 1, idx_waypoint_behind_car + d_index);
  return {waypoints[idx].x, waypoints[idx].y};
}

PpOutput PpCore::mainLoop(const PpInput & in)
{
  // pp.py :98-139 (the speed vector v = [cos(yaw)*speed, sin(yaw)*speed] is
  // recomputed inside calcSpeedCommand/calcSteeringAngle instead of threaded)

  // calc_lateral_error_norm (:350-365). With max_lat_e=0.5, min_lat_e=0 the
  // expression reduces to lat_e_norm == clip(|d|, 0, 0.5); ported literally.
  const double lateral_error = std::abs(in.d);
  const double max_lat_e = 0.5;
  const double min_lat_e = 0.0;
  const double lat_e_clip = npClip(lateral_error, min_lat_e, max_lat_e);
  const double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));

  /// LONGITUDINAL ///
  const double speed_command = calcSpeedCommand(in, lat_e_norm);
  // :120-128 -- np.max(speed_command, 0): the 0 is the AXIS argument, not a
  // floor, so this is the identity on a scalar. Port literally, do not "fix".
  const double speed = speed_command;
  const double acceleration = 0.0;
  const double jerk = 0.0;

  /// LATERAL ///
  double l1_distance = 0.0;
  const auto l1_point = calcL1Point(in, lateral_error, &l1_distance);
  // :134-137 `if L1_point.any() is not None` is always true -> always steer
  const double steering_angle =
    calcSteeringAngle(in, l1_point.first, l1_point.second, l1_distance, lat_e_norm);

  PpOutput out;
  out.speed = speed;
  out.acceleration = acceleration;
  out.jerk = jerk;
  out.steering_angle = steering_angle;
  out.l1_x = l1_point.first;
  out.l1_y = l1_point.second;
  out.l1_distance = l1_distance;
  out.idx_nearest_waypoint = idx_nearest_waypoint_;
  out.trailing_ran = in.is_trailing && in.opponent.has_value();
  out.gap = gap_;
  out.gap_should = gap_should_;
  out.gap_error = gap_error_;
  out.v_diff = v_diff_;
  out.i_gap = i_gap_;
  out.trailing_command = trailing_command_;
  return out;
}

double PpCore::calcSpeedCommand(const PpInput & in, double lat_e_norm)
{
  // pp.py :244-271
  const std::vector<LocalWaypoint> & wpts = *in.waypoints;
  const double adv_ts_sp = p_.speed_lookahead;
  const double la_x = in.x + std::cos(in.yaw) * in.speed_now * adv_ts_sp;
  const double la_y = in.y + std::sin(in.yaw) * in.speed_now * adv_ts_sp;
  const int idx_la_position = nearestWaypoint(la_x, la_y, wpts);
  const double global_speed = wpts[idx_la_position].v;

  double speed_command;
  if (in.is_trailing && in.opponent.has_value()) {
    speed_command = trailingController(in, global_speed);
  } else {
    i_gap_ = 0.0;
    speed_command = global_speed;
  }

  speed_command = speedAdjustLatErr(speed_command, lat_e_norm);
  return speed_command;
}

double PpCore::trailingController(const PpInput & in, double global_speed)
{
  // pp.py :273-316
  const Opponent & opp = *in.opponent;
  gap_ = pyMod(opp.s_center - in.s, in.track_length);  // python % is non-negative
  const double gap_actual = gap_;
  // speed-scaled gap; trailing_vel_gain 0.0 disables the scaling
  gap_should_ = p_.trailing_gap + p_.trailing_vel_gain * in.speed_now;
  gap_error_ = gap_should_ - gap_actual;
  v_diff_ = in.vs - opp.vs;
  i_gap_ = npClip(i_gap_ + gap_error_ / p_.loop_rate, -10.0, 10.0);

  const double p_value = gap_error_ * p_.trailing_p_gain;
  const double d_value = v_diff_ * p_.trailing_d_gain;
  const double i_value = i_gap_ * p_.trailing_i_gain;

  trailing_command_ = npClip(opp.vs - p_value - i_value - d_value, 0.0, global_speed);
  if (!opp.is_visible && gap_actual > gap_should_) {
    trailing_command_ = std::max(p_.blind_trailing_speed, trailing_command_);
  } else if (gap_actual > gap_should_) {
    // creep floor for VISIBLE targets while gap > gap_should
    trailing_command_ = std::max(p_.trailing_min_speed, trailing_command_);
  } else if (p_.trailing_creep_always &&
    gap_actual - p_.trailing_nose_offset - 0.5 * opp.size > p_.trailing_stop_gap)
  {
    // always-creep: keep the floor even AT/inside the desired gap; only truly
    // stops below trailing_stop_gap of BUMPER clearance
    trailing_command_ = std::max(p_.trailing_min_speed, trailing_command_);
  }
  return trailing_command_;
}

double PpCore::speedAdjustLatErr(double global_speed, double lat_e_norm) const
{
  // pp.py :367-383 (lat_e_norm is passed by value in python too -- the *=2
  // does not leak back to the caller)
  const double lat_e_coeff = p_.lat_err_coeff;
  lat_e_norm *= 2.0;
  const double curv = npClip(2.0 * (curvature_waypoints_ / 0.8) - 2.0, 0.0, 1.0);
  global_speed *= (1.0 - lat_e_coeff + lat_e_coeff * std::exp(-lat_e_norm * curv));
  return global_speed;
}

std::pair<double, double> PpCore::calcL1Point(
  const PpInput & in, double lateral_error, double * l1_distance_out)
{
  // pp.py :193-241
  const std::vector<LocalWaypoint> & wpts = *in.waypoints;
  idx_nearest_waypoint_ = nearestWaypoint(in.x, in.y, wpts);

  // curvature mean over the REMAINING local waypoints; only recomputed when
  // more than 2 points remain past the car, else the previous value persists
  const int n_remaining = static_cast<int>(wpts.size()) - idx_nearest_waypoint_;
  if (n_remaining > 2) {
    double sum = 0.0;
    for (std::size_t i = idx_nearest_waypoint_; i < wpts.size(); ++i) {
      sum += std::abs(wpts[i].kappa);
    }
    curvature_waypoints_ = sum / static_cast<double>(n_remaining);
  }

  double l1_distance = p_.q_l1 + in.speed_now * p_.m_l1;

  // clip lower bound to avoid ultraswerve when far away from mincurv
  const double lower_bound = std::max(p_.t_clip_min, std::sqrt(2.0) * lateral_error);
  l1_distance = npClip(l1_distance, lower_bound, p_.t_clip_max);

  // RECOVERY: extend lookahead ~ speed of the lookahead waypoint (:224-229)
  if (in.is_recovery && p_.recovery_l1_gain > 0.0) {
    const int idx_la = static_cast<int>(std::min(
        static_cast<double>(idx_nearest_waypoint_) + l1_distance / 0.1 + 0.5,
        static_cast<double>(wpts.size()) - 1.0));
    const double v_la = wpts[idx_la].v;
    l1_distance = std::min(
      l1_distance + p_.recovery_l1_gain * std::max(v_la, 0.0),
      p_.recovery_t_clip_max);
  }

  // hairpin cap (:231-238): applied AFTER the recovery extension so the cap wins via min()
  if (p_.l1_curv_cap_enable) {
    if (static_cast<int>(wpts.size()) - idx_nearest_waypoint_ > 0) {
      double heading_cum = 0.0;
      for (std::size_t i = idx_nearest_waypoint_; i < wpts.size(); ++i) {
        heading_cum += std::abs(wpts[i].kappa) * 0.1;
        if (heading_cum > p_.l1_curv_cap_max_heading) {
          const double over0 =
            static_cast<double>(i - static_cast<std::size_t>(idx_nearest_waypoint_));
          l1_distance = std::min(l1_distance, std::max(over0 * 0.1, lower_bound));
          break;
        }
      }
    }
  }

  *l1_distance_out = l1_distance;
  return waypointAtDistanceBeforeCar(l1_distance, wpts, idx_nearest_waypoint_);
}

double PpCore::calcSteeringAngle(
  const PpInput & in, double l1_x, double l1_y, double l1_distance,
  double lat_e_norm)
{
  // pp.py :141-191
  const std::vector<LocalWaypoint> & wpts = *in.waypoints;

  double speed_la_for_lu;
  if (in.is_trailing && in.opponent.has_value()) {
    speed_la_for_lu = in.speed_now;
  } else {
    const double adv_ts_st = p_.speed_lookahead_for_steer;
    const double la_x = in.x + std::cos(in.yaw) * in.speed_now * adv_ts_st;
    const double la_y = in.y + std::sin(in.yaw) * in.speed_now * adv_ts_st;
    const int idx_la_steer = nearestWaypoint(la_x, la_y, wpts);
    speed_la_for_lu = wpts[idx_la_steer].v;
  }
  const double speed_for_lu = speedAdjustLatErr(speed_la_for_lu, lat_e_norm);

  const double l1_vec_x = l1_x - in.x;
  const double l1_vec_y = l1_y - in.y;
  const double l1_norm = std::sqrt(l1_vec_x * l1_vec_x + l1_vec_y * l1_vec_y);
  double eta;
  if (l1_norm == 0.0) {
    eta = 0.0;  // python logs a warning here; the node has no access -> silent
  } else {
    eta = std::asin(
      (-std::sin(in.yaw) * l1_vec_x + std::cos(in.yaw) * l1_vec_y) / l1_norm);
  }

  double steering_angle = std::atan(2.0 * p_.wheelbase * std::sin(eta) / l1_distance);

  // acc_scaling (:322-335): thresholds on the MEAN of the imu buffer
  if (in.acc_mean >= 1.0) {
    steering_angle *= p_.acc_scaler_for_steer;
  } else if (in.acc_mean <= -1.0) {
    steering_angle *= p_.dec_scaler_for_steer;
  }

  // speed_steer_scaling (:337-348)
  const double speed_diff = std::max(0.1, p_.end_scale_speed - p_.start_scale_speed);
  const double factor = 1.0 -
    npClip((speed_for_lu - p_.start_scale_speed) / speed_diff, 0.0, 1.0) *
    p_.downscale_factor;
  steering_angle *= factor;

  // modifying steer based on velocity (:183)
  steering_angle *= npClip(1.0 + in.speed_now / 10.0, 1.0, 1.25);

  // limit change of steering angle (:186-190)
  const double threshold = 0.4;
  steering_angle = npClip(
    steering_angle, curr_steering_angle_ - threshold, curr_steering_angle_ + threshold);
  curr_steering_angle_ = steering_angle;
  return steering_angle;
}

}  // namespace controller
