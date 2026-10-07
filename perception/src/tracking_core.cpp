#include <perception/tracking_core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace perception
{

double pyMod(double value, double modulus)
{
  const double result = std::fmod(value, modulus);
  if (result != 0.0 && ((result < 0.0) != (modulus < 0.0))) {
    return result + modulus;
  }
  return result;
}

double normalizeS(double s, double track_length)
{
  double new_s = pyMod(s, track_length);
  if (new_s > track_length / 2.0) {
    new_s -= track_length;
  }
  return new_s;
}

// ---------------------------------------------------------------------------
// TrackedObstacle (python ObstacleSD)
// ---------------------------------------------------------------------------

TrackedObstacle::TrackedObstacle(
  int id_, double s_meas, double d_meas, int lap, double size_, bool is_visible_, int ttl_)
: id(id_), measurements_s{s_meas}, measurements_d{d_meas}, mean{{s_meas, d_meas}}, ttl(ttl_),
  current_lap(lap), size(size_), is_visible(is_visible_)
{
}

void TrackedObstacle::updateMean(double track_length)
{
  if (nb_meas == 0) {
    mean = {measurements_s.back(), measurements_d.back()};
    return;
  }
  // running mean over ALL measurements ever seen (nb_meas), not only the kept window
  mean[1] = (mean[1] * nb_meas + measurements_d.back()) / (nb_meas + 1);

  // s is averaged on the unit circle so that the track wrap-around is handled
  const double previous_mean_rad = mean[0] * 2.0 * M_PI / track_length;
  const double current_meas_rad = measurements_s.back() * 2.0 * M_PI / track_length;
  const double cos_mean_angle =
    (std::cos(previous_mean_rad) * nb_meas + std::cos(current_meas_rad)) / (nb_meas + 1);
  const double sin_mean_angle =
    (std::sin(previous_mean_rad) * nb_meas + std::sin(current_meas_rad)) / (nb_meas + 1);
  const double mean_angle = std::atan2(sin_mean_angle, cos_mean_angle);
  const double mean_s = mean_angle * track_length / 2.0 / M_PI;
  mean[0] = mean_s >= 0.0 ? mean_s : mean_s + track_length;
}

double TrackedObstacle::stdS(double track_length) const
{
  double sum = 0.0;
  for (const double s : measurements_s) {
    const double diff = normalizeS(s - mean[0], track_length);
    sum += diff * diff;
  }
  return std::sqrt(sum / static_cast<double>(measurements_s.size()));
}

double TrackedObstacle::stdD() const
{
  // numpy.std: population standard deviation
  const double n = static_cast<double>(measurements_d.size());
  double mean_d = 0.0;
  for (const double d : measurements_d) {
    mean_d += d;
  }
  mean_d /= n;
  double sum = 0.0;
  for (const double d : measurements_d) {
    sum += (d - mean_d) * (d - mean_d);
  }
  return std::sqrt(sum / n);
}

void TrackedObstacle::classifyStatic(const TrackingParams & params, double track_length)
{
  if (nb_meas > params.min_nb_meas) {
    const double std_s = stdS(track_length);
    const double std_d = stdD();
    // voting so that outliers do not flip the classification
    if (std_s < params.min_std && std_d < params.min_std) {
      static_count += 1;
    } else if (std_s > params.max_std || std_d > params.max_std) {
      static_count = 0;
    }
    total_count += 1;
    static_flag = (static_cast<double>(static_count) / total_count >= 0.5) ?
      StaticFlag::Static : StaticFlag::Dynamic;
  } else {
    static_flag = StaticFlag::Unknown;
  }
}

// ---------------------------------------------------------------------------
// OpponentState (python Opponent_state)
// ---------------------------------------------------------------------------

namespace
{

Eigen::Matrix2d discreteWhiteNoise(double dt, double var)
{
  // filterpy Q_discrete_white_noise(dim=2)
  Eigen::Matrix2d q;
  q << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt,
    0.5 * dt * dt * dt, dt * dt;
  return q * var;
}

}  // namespace

void OpponentState::configure(const TrackingParams & params)
{
  rate_hz_ = params.rate_hz;
  const double dt = 1.0 / params.rate_hz;
  F_ << 1.0, dt, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, dt,
    0.0, 0.0, 0.0, 1.0;
  Q_.setZero();
  Q_.block<2, 2>(0, 0) = discreteWhiteNoise(dt, params.process_var_vs);
  Q_.block<2, 2>(2, 2) = discreteWhiteNoise(dt, params.process_var_vd);
  R_ = Eigen::Vector4d(
    params.measurement_var_s, params.measurement_var_vs,
    params.measurement_var_d, params.measurement_var_vd).asDiagonal();
  P = Eigen::Vector4d(
    params.measurement_var_s, params.process_var_vs,
    params.measurement_var_d, params.process_var_vd).asDiagonal();
  x.setZero();
  vs_filt.fill(0.0);
  vd_filt.fill(0.0);
}

double OpponentState::targetVelocity(
  const TrackingParams & params, const std::vector<f110_msgs::msg::Wpnt> & waypoints,
  double track_length) const
{
  // PY-QUIRK: python indexes waypoints with int((x[0]*10) % track_length); reproduced
  // as-is, with the index clamped to the waypoint count.
  if (waypoints.empty()) {
    return 0.0;
  }
  const double raw_index = pyMod(x[0] * 10.0, track_length);
  std::size_t index = static_cast<std::size_t>(std::max(0.0, raw_index));
  index = std::min(index, waypoints.size() - 1);
  return params.ratio_to_glob_path * waypoints[index].vx_mps;
}

void OpponentState::predict(
  const TrackingParams & params, const std::vector<f110_msgs::msg::Wpnt> & waypoints,
  double track_length)
{
  Eigen::Vector4d u;
  if (use_target_vel) {
    u << 0.0,
      params.p_vs * (targetVelocity(params, waypoints, track_length) - x[1]),
      -params.p_d * x[2],
      -params.p_vd * x[3];
  } else {
    u << 0.0, 0.0, -params.p_d * x[2], -params.p_vd * x[3];
  }
  // filterpy: x = F x + B u (B = I), P = F P F' + Q
  x = F_ * x + u;
  P = F_ * P * F_.transpose() + Q_;
  x[0] = normalizeS(x[0], track_length);
}

void OpponentState::update(
  const TrackingParams & /*params*/, const TrackedObstacle & tracked, double track_length)
{
  const auto & ms = tracked.measurements_s;
  const auto & md = tracked.measurements_d;
  const std::size_t n = ms.size();
  if (n < 3U) {
    // python indexes [-3]; a dynamic obstacle always has > min_nb_meas samples
    throw std::logic_error("OpponentState::update needs at least 3 measurements");
  }
  const double rate = rate_hz_;
  const double vs = (2.0 / 3.0 * (ms[n - 1] - ms[n - 2]) * rate) +
    (1.0 / 3.0 * (ms[n - 2] - ms[n - 3]) * rate);

  if (!(vs > -1.0 && vs < 8.0)) {
    is_initialised = false;
    return;
  }

  Eigen::Vector4d z;
  z << normalizeS(ms[n - 1], track_length), vs, md[n - 1], (md[n - 1] - md[n - 2]) * rate;

  // filterpy ExtendedKalmanFilter.update with H = I, hx = [normalize_s(x0), x1, x2, x3]
  const Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d PHT = P * H.transpose();
  const Eigen::Matrix4d S = H * PHT + R_;
  const Eigen::Matrix4d K = PHT * Eigen::PartialPivLU<Eigen::Matrix4d>(S).inverse();
  Eigen::Vector4d hx = x;
  hx[0] = normalizeS(x[0], track_length);
  Eigen::Vector4d y = z - hx;
  y[0] = normalizeS(y[0], track_length);
  x = x + K * y;
  const Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K * H;
  P = I_KH * P * I_KH.transpose() + K * R_ * K.transpose();
  x[0] = normalizeS(x[0], track_length);

  vs_list.push_back(x[1]);
  if (vs_list.size() > 20U) {
    vs_list.erase(vs_list.begin(), vs_list.end() - 10);
  }
  avg_vs = 0.0;
  for (const double v : vs_list) {
    avg_vs += v;
  }
  avg_vs /= static_cast<double>(vs_list.size());

  // PY-QUIRK: python sets filt[0] = v and THEN does filt[1:] = filt[:-1], so the
  // newest value ends up twice at the head ([v, v, old0, old1, old2]).
  vs_filt[0] = x[1];
  for (std::size_t i = vs_filt.size() - 1; i >= 1; --i) {
    vs_filt[i] = vs_filt[i - 1];
  }
  vd_filt[0] = x[3];
  for (std::size_t i = vd_filt.size() - 1; i >= 1; --i) {
    vd_filt[i] = vd_filt[i - 1];
  }

  if (vs_list.size() >= 10U) {
    vs_list.erase(vs_list.begin());
  }
  vs_list.push_back(x[1]);
}

void OpponentState::initialise(const TrackingParams & params, const TrackedObstacle & tracked)
{
  const auto & ms = tracked.measurements_s;
  const auto & md = tracked.measurements_d;
  const std::size_t n = ms.size();
  if (n < 2U) {
    throw std::logic_error("OpponentState::initialise needs at least 2 measurements");
  }
  // PY-QUIRK: P is NOT reset here (python only assigns x), so the covariance of a
  // previous opponent carries over into the new one.
  x << ms[n - 1], (ms[n - 1] - ms[n - 2]) * params.rate_hz,
    md[n - 1], (md[n - 1] - md[n - 2]) * params.rate_hz;
  is_initialised = true;
  id = tracked.id;
  ttl = params.ttl_dynamic;
  size = tracked.size;
  avg_vs = 0.0;
  vs_list.clear();
}

double OpponentState::meanVsFilt() const
{
  double sum = 0.0;
  for (const double v : vs_filt) {
    sum += v;
  }
  return sum / static_cast<double>(vs_filt.size());
}

double OpponentState::meanVdFilt() const
{
  double sum = 0.0;
  for (const double v : vd_filt) {
    sum += v;
  }
  return sum / static_cast<double>(vd_filt.size());
}

// ---------------------------------------------------------------------------
// Tracker (python StaticDynamic)
// ---------------------------------------------------------------------------

Tracker::Tracker(const TrackingParams & params)
: params_(params)
{
  if (!(params_.rate_hz > 0.0)) {
    throw std::invalid_argument("rate must be positive");
  }
  opponent_.configure(params_);
}

bool Tracker::setGlobalPath(const std::vector<f110_msgs::msg::Wpnt> & waypoints)
{
  if (initialized_track_bounds_) {
    return false;
  }
  if (waypoints.size() < 2U) {
    return false;
  }
  converter_.setGlobalTrajectory(waypoints, true);  // throws on a non-positive length
  global_path_ = waypoints;
  track_length_ = waypoints.back().s_m;
  initialized_track_bounds_ = true;
  return true;
}

void Tracker::setCarS(double s)
{
  car_s_ = s;
  if (!last_car_s_) {
    last_car_s_ = s;
  }
}

void Tracker::setCarPose(double x, double y, double yaw)
{
  car_x_ = x;
  car_y_ = y;
  car_cos_yaw_ = std::cos(yaw);
  car_sin_yaw_ = std::sin(yaw);
}

void Tracker::setScan(ScanData scan)
{
  scan_ = std::move(scan);
}

void Tracker::setMeasurements(std::vector<f110_msgs::msg::Obstacle> obstacles)
{
  meas_obstacles_ = std::move(obstacles);
}

void Tracker::step()
{
  if (opponent_.is_initialised) {
    opponent_.predict(params_, global_path_, track_length_);
  }
  update();
}

void Tracker::update()
{
  if (!car_s_ || !initialized_track_bounds_) {
    return;
  }

  const std::vector<f110_msgs::msg::Obstacle> meas_copy = meas_obstacles_;
  std::vector<const f110_msgs::msg::Obstacle *> candidates;
  candidates.reserve(meas_copy.size());
  for (const auto & meas : meas_copy) {
    candidates.push_back(&meas);
  }
  const double car_s = *car_s_;
  const double car_x = car_x_;
  const double car_y = car_y_;
  const double cos_yaw = car_cos_yaw_;
  const double sin_yaw = car_sin_yaw_;

  lapUpdate(car_s);

  std::vector<bool> remove(tracked_obstacles_.size(), false);
  for (std::size_t index = 0; index < tracked_obstacles_.size(); ++index) {
    TrackedObstacle & tracked = tracked_obstacles_[index];
    const f110_msgs::msg::Obstacle * meas = verifyPosition(tracked, candidates);

    if (meas != nullptr) {
      updateTrackedObstacle(tracked, *meas);

      if (tracked.static_flag == StaticFlag::Dynamic) {
        if (opponent_.is_initialised) {
          opponent_.use_target_vel = false;
          if (opponent_.avg_vs < params_.vs_reset && opponent_.vs_list.size() > 10U &&
            params_.publish_static)
          {
            opponent_.is_initialised = false;
            tracked.static_flag = StaticFlag::Static;
            tracked.static_count = 0;
            tracked.total_count = 0;
            tracked.nb_meas = 0;
          } else {
            opponent_.update(params_, tracked, track_length_);
            opponent_.id = tracked.id;
            opponent_.ttl = params_.ttl_dynamic;
            opponent_.size = tracked.size;
          }
        } else {
          opponent_.initialise(params_, tracked);
        }
      }

      // assigned: remove from further association
      candidates.erase(std::find(candidates.begin(), candidates.end(), meas));
    } else {
      if (tracked.ttl <= 0) {
        if (tracked.static_flag == StaticFlag::Dynamic) {
          opponent_.use_target_vel = true;
        }
        remove[index] = true;
      } else if (tracked.static_flag == StaticFlag::Unknown) {
        tracked.ttl -= 1;
      } else {
        tracked.is_in_front = checkInFront(tracked, car_s);
        const double distance_obstacle_car = distanceObsCar(tracked, car_s);
        const bool is_static = tracked.static_flag == StaticFlag::Static;

        if (is_static && params_.no_memory_mode) {
          tracked.ttl -= 1;
        } else if (distance_obstacle_car < params_.dist_deletion && is_static) {
          frenet_conversion::GlobalPoint point;
          try {
            point = converter_.getGlobalPoint(tracked.mean[0], tracked.mean[1]);
          } catch (const std::exception &) {
            continue;
          }
          if (checkInFieldOfView(point.x - car_x, point.y - car_y, cos_yaw, sin_yaw)) {
            tracked.ttl -= 1;
            tracked.is_visible = true;
          } else {
            tracked.is_visible = false;
          }
        } else if (!is_static) {
          tracked.ttl -= 1;
        } else {
          tracked.is_visible = false;
        }
      }
    }
  }

  if (opponent_.is_initialised) {
    if (opponent_.ttl <= 0) {
      opponent_.is_initialised = false;
      opponent_.use_target_vel = false;
    } else {
      opponent_.ttl -= 1;
    }
  }

  std::vector<TrackedObstacle> kept;
  kept.reserve(tracked_obstacles_.size());
  for (std::size_t index = 0; index < tracked_obstacles_.size(); ++index) {
    if (!remove[index]) {
      kept.push_back(std::move(tracked_obstacles_[index]));
    }
  }
  tracked_obstacles_ = std::move(kept);

  for (const auto * meas : candidates) {
    tracked_obstacles_.emplace_back(
      current_id_, meas->s_center, meas->d_center, current_lap_, meas->size, true,
      params_.ttl_static);
    current_id_ += 1;
  }
}

void Tracker::lapUpdate(double car_s)
{
  if (!last_car_s_) {
    return;
  }
  if (car_s - *last_car_s_ < -track_length_ / 2.0) {
    current_lap_ += 1;
  }
  last_car_s_ = car_s;
}

const f110_msgs::msg::Obstacle * Tracker::closestCandidate(
  double max_dist, double s, double d,
  const std::vector<const f110_msgs::msg::Obstacle *> & candidates) const
{
  // python get_closest_pos + argmin: first minimum wins on ties
  const f110_msgs::msg::Obstacle * best = nullptr;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto * meas : candidates) {
    const double dist = std::hypot(s - meas->s_center, d - meas->d_center);
    if (dist < max_dist && dist < best_dist) {
      best_dist = dist;
      best = meas;
    }
  }
  return best;
}

const f110_msgs::msg::Obstacle * Tracker::verifyPosition(
  const TrackedObstacle & obstacle,
  const std::vector<const f110_msgs::msg::Obstacle *> & candidates) const
{
  double max_dist = params_.max_dist;
  double s = obstacle.mean[0];
  double d = obstacle.mean[1];
  if (obstacle.static_flag == StaticFlag::Dynamic) {
    // dynamic obstacles are associated with the KF prediction for better accuracy
    s = pyMod(opponent_.x[0], track_length_);
    d = opponent_.x[2];
    max_dist *= params_.aggro_multiplier;
  }

  const auto * match = closestCandidate(max_dist, s, d, candidates);
  if (match != nullptr) {
    return match;
  }
  // maybe the kalman filter was wrong: retry with the obstacle mean
  if (obstacle.static_flag == StaticFlag::Dynamic) {
    return closestCandidate(max_dist, obstacle.mean[0], obstacle.mean[1], candidates);
  }
  return nullptr;
}

void Tracker::updateTrackedObstacle(
  TrackedObstacle & tracked, const f110_msgs::msg::Obstacle & meas)
{
  tracked.measurements_s.push_back(meas.s_center);
  tracked.measurements_d.push_back(meas.d_center);

  if (tracked.measurements_s.size() > 30U) {
    tracked.measurements_s.erase(
      tracked.measurements_s.begin(), tracked.measurements_s.end() - 20);
    tracked.measurements_d.erase(
      tracked.measurements_d.begin(), tracked.measurements_d.end() - 20);
  }

  tracked.updateMean(track_length_);
  tracked.nb_meas += 1;
  tracked.is_in_front = true;
  tracked.is_visible = true;
  tracked.current_lap = current_lap_;
  tracked.size = meas.size;
  tracked.classifyStatic(params_, track_length_);
  tracked.ttl = params_.ttl_static;
}

bool Tracker::checkInFront(const TrackedObstacle & tracked, double car_s) const
{
  const double obj_dist_in_front = normalizeS(tracked.measurements_s.back() - car_s, track_length_);
  return 0.0 < obj_dist_in_front && obj_dist_in_front < params_.dist_infront;
}

double Tracker::distanceObsCar(const TrackedObstacle & tracked, double car_s) const
{
  return pyMod(tracked.measurements_s.back() - car_s, track_length_);
}

double Tracker::angleToObstacle(double vec_x, double vec_y, double cos_yaw, double sin_yaw)
{
  // rotate the map-frame vector into the car frame
  const double x = cos_yaw * vec_x + sin_yaw * vec_y;
  const double y = -sin_yaw * vec_x + cos_yaw * vec_y;
  return std::atan2(y, x);
}

bool Tracker::checkInFieldOfView(
  double vec_x, double vec_y, double cos_yaw, double sin_yaw) const
{
  const double dist_to_obs = std::hypot(vec_x, vec_y);
  const double bearing_angle = angleToObstacle(vec_x, vec_y, cos_yaw, sin_yaw);

  if (bearing_angle > scan_.angle_max || bearing_angle < scan_.angle_min) {
    return false;
  }
  if (scan_.angle_increment == 0.0) {
    return false;  // python would divide by zero (no scan received yet)
  }
  // python: int(round(x)) -> round half to even
  const long obstacle_scan_idx =
    std::lrint((bearing_angle - scan_.angle_min) / scan_.angle_increment);
  const long largest_scan_idx = static_cast<long>(scan_.ranges.size());
  if (obstacle_scan_idx < 0 || obstacle_scan_idx >= largest_scan_idx) {
    return false;
  }

  // slice of beams around the expected bearing: [idx-4, idx+4)
  const long low_index = std::max(0L, obstacle_scan_idx - 4);
  const long high_index = std::min(obstacle_scan_idx + 4, largest_scan_idx);
  // python min(): first element wins unless a later one compares smaller (NaN-safe in the same way)
  double min_range = static_cast<double>(scan_.ranges[static_cast<std::size_t>(low_index)]);
  for (long i = low_index + 1; i < high_index; ++i) {
    const double r = static_cast<double>(scan_.ranges[static_cast<std::size_t>(i)]);
    if (r < min_range) {
      min_range = r;
    }
  }
  // margin: beams reaching (almost) as far as the remembered position count as
  // seeing that spot (wall-adjacent ghosts would otherwise be immortal)
  return dist_to_obs - params_.fov_dist_margin < min_range;
}

void Tracker::buildObstacleArrays(
  std::vector<f110_msgs::msg::Obstacle> & estimated,
  std::vector<f110_msgs::msg::Obstacle> & raw_opponent) const
{
  estimated.clear();
  raw_opponent.clear();
  const double L = track_length_;

  for (const auto & obs : tracked_obstacles_) {
    f110_msgs::msg::Obstacle msg;
    msg.id = obs.id;
    msg.size = obs.size;
    msg.vs = 0.0;
    msg.vd = 0.0;
    msg.is_static = true;
    msg.is_actually_a_gap = false;
    msg.is_visible = obs.is_visible;

    if (obs.static_flag == StaticFlag::Static) {
      msg.s_center = obs.mean[0];
      msg.d_center = obs.mean[1];
    } else {
      msg.s_center = pyMod(obs.measurements_s.back(), L);
      msg.d_center = obs.measurements_d.back();
    }
    // python precedence: s_center - (size/2 % L)
    msg.s_start = msg.s_center - pyMod(msg.size / 2.0, L);
    msg.s_end = msg.s_center + pyMod(msg.size / 2.0, L);
    msg.d_right = msg.d_center - msg.size / 2.0;
    msg.d_left = msg.d_center + msg.size / 2.0;

    if (obs.static_flag != StaticFlag::Dynamic && params_.publish_static) {
      estimated.push_back(msg);
    } else {
      raw_opponent.push_back(msg);
    }
  }

  if (opponent_.is_initialised && opponent_.P(0, 0) < params_.var_pub) {
    f110_msgs::msg::Obstacle msg;
    msg.id = opponent_.id;
    msg.size = opponent_.size;
    msg.vs = opponent_.meanVsFilt();
    msg.vd = opponent_.meanVdFilt();
    msg.is_static = false;
    msg.is_actually_a_gap = false;
    msg.is_visible = true;
    msg.s_center = pyMod(opponent_.x[0], L);
    msg.d_center = opponent_.x[2];
    msg.s_start = msg.s_center - pyMod(msg.size / 2.0, L);
    msg.s_end = msg.s_center + pyMod(msg.size / 2.0, L);
    msg.d_right = msg.d_center - msg.size / 2.0;
    msg.d_left = msg.d_center + msg.size / 2.0;
    estimated.push_back(msg);
  }
}

}  // namespace perception
