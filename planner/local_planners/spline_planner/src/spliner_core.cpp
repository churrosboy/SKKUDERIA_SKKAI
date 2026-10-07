#include <spline_planner/spliner_core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace spline_planner
{

double pyMod(double value, double modulus)
{
  const double result = std::fmod(value, modulus);
  if (result != 0.0 && ((result < 0.0) != (modulus < 0.0))) {
    return result + modulus;
  }
  return result;
}

namespace
{

std::string format(const char * fmt, double a, double b, double c = 0.0, double d = 0.0)
{
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), fmt, a, b, c, d);
  return buffer;
}

}  // namespace

Spliner::Spliner(const SplinerParams & params, Logger logger)
: params_(params), logger_(std::move(logger))
{
}

void Spliner::log(LogLevel level, const std::string & message) const
{
  if (logger_) {
    logger_(level, message);
  }
}

void Spliner::setGlobalPath(const std::vector<f110_msgs::msg::Wpnt> & wpnts)
{
  if (wpnts.size() < 2U) {
    throw std::invalid_argument("global path needs >= 2 waypoints");
  }
  if (!has_global_) {
    // python gb_cb: vmax / max idx / max s are taken from the FIRST message only,
    // and the converter is built once from that message
    gb_vmax_ = wpnts.front().vx_mps;
    for (const auto & w : wpnts) {
      gb_vmax_ = std::max(gb_vmax_, w.vx_mps);
    }
    gb_max_idx_ = wpnts.back().id;
    gb_max_s_ = wpnts.back().s_m;
    std::vector<double> x, y;
    x.reserve(wpnts.size());
    y.reserve(wpnts.size());
    for (const auto & w : wpnts) {
      x.push_back(w.x_m);
      y.push_back(w.y_m);
    }
    converter_ = PyFrenetConverter(x, y);
    has_global_ = true;
  }
}

void Spliner::setScaledPath(const std::vector<f110_msgs::msg::Wpnt> & wpnts)
{
  // /global_waypoints_scaled is republished every 0.5 s by sector_tuner with
  // identical content -> only a real change (vx / bounds / lane splice) drops the cache
  if (!cache_.empty() && wpnts != gb_scaled_wpnts_) {
    clearCache();
  }
  gb_scaled_wpnts_ = wpnts;
  has_scaled_ = !wpnts.empty();
}

// ---- evasion cache ---------------------------------------------------------------------

void Spliner::clearCache()
{
  cache_.clear();
}

bool Spliner::cachePositionMatches(
  const EvasionCacheEntry & entry, const f110_msgs::msg::Obstacle & merged) const
{
  const double L = gb_max_s_;
  const double tol = params_.cache_pos_tol_m;
  auto ring_diff = [&](double a, double b) {
      return std::abs(pyMod(a - b + L / 2.0, L) - L / 2.0);
    };
  return ring_diff(entry.s_start, merged.s_start) < tol &&
         ring_diff(entry.s_end, merged.s_end) < tol &&
         std::abs(entry.d_left - merged.d_left) < tol &&
         std::abs(entry.d_right - merged.d_right) < tol;
}

EvasionCacheEntry * Spliner::findCacheEntry(const f110_msgs::msg::Obstacle & merged)
{
  for (auto & entry : cache_) {
    if (cachePositionMatches(entry, merged) &&
      std::abs(entry.vs_at_compute - cur_vs_) < params_.cache_vs_tol_mps)
    {
      return &entry;
    }
  }
  return nullptr;
}

void Spliner::storeCacheEntry(EvasionCacheEntry entry)
{
  entry.last_use = ++cache_tick_;
  // one slot per obstacle position: a speed-tolerance miss overwrites instead of appending
  for (auto & existing : cache_) {
    f110_msgs::msg::Obstacle as_obs;
    as_obs.s_start = entry.s_start;
    as_obs.s_end = entry.s_end;
    as_obs.d_left = entry.d_left;
    as_obs.d_right = entry.d_right;
    if (cachePositionMatches(existing, as_obs)) {
      existing = std::move(entry);
      return;
    }
  }
  const std::size_t max_entries = static_cast<std::size_t>(std::max(1, params_.cache_max_entries));
  if (cache_.size() >= max_entries) {
    auto lru = std::min_element(
      cache_.begin(), cache_.end(),
      [](const auto & a, const auto & b) {return a.last_use < b.last_use;});
    *lru = std::move(entry);
    return;
  }
  cache_.push_back(std::move(entry));
}

void Spliner::setState(double s, double d, double vs)
{
  cur_s_ = s;
  cur_d_ = d;
  cur_vs_ = vs;
  has_state_ = true;
}

void Spliner::setObstacles(std::vector<f110_msgs::msg::Obstacle> obstacles)
{
  obs_ = std::move(obstacles);
}

SplinerOutput Spliner::step(bool viz_on, bool want_propagated)
{
  if (!ready()) {
    throw std::logic_error("Spliner::step before all inputs arrived");
  }
  if (!obs_.empty()) {
    return doSpline(obs_, gb_scaled_wpnts_, viz_on, want_propagated);
  }
  SplinerOutput out;
  out.delete_markers = true;
  return out;
}

f110_msgs::msg::Obstacle Spliner::predictObsMovement(
  f110_msgs::msg::Obstacle obs, SplinerOutput & out, bool want_propagated) const
{
  // PY-QUIRK(fixed): python mutated the obstacle inside the stored ObstacleArray, so a
  // message processed by two loop cycles was propagated twice. We work on a copy.
  if (pyMod(obs.s_center - cur_s_, gb_max_s_) < 10.0) {
    const double delta_s = params_.fixed_pred_time * obs.vs;
    const double delta_d = params_.fixed_pred_time * obs.vd;
    obs.s_start += delta_s;
    obs.s_center += delta_s;
    obs.s_end += delta_s;
    obs.s_start = pyMod(obs.s_start, gb_max_s_);
    obs.s_center = pyMod(obs.s_center, gb_max_s_);
    obs.s_end = pyMod(obs.s_end, gb_max_s_);
    obs.d_left += delta_d;
    obs.d_center += delta_d;
    obs.d_right += delta_d;
    if (want_propagated) {
      const auto p = converter_.getCartesian(obs.s_center, obs.d_center);
      out.propagated.push_back({p.x, p.y, 0.0});
    }
  }
  return obs;
}

std::vector<f110_msgs::msg::Obstacle> Spliner::obsFiltering(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles, SplinerOutput & out,
  bool want_propagated) const
{
  std::vector<f110_msgs::msg::Obstacle> close_obs;
  for (const auto & raw : obstacles) {
    if (!(std::abs(raw.d_center) < params_.obs_traj_tresh)) {
      continue;
    }
    const auto obs = predictObsMovement(raw, out, want_propagated);
    const double dist_in_front = pyMod(obs.s_center - cur_s_, gb_max_s_);
    if (dist_in_front < params_.lookahead) {
      close_obs.push_back(obs);
    }
  }
  return close_obs;
}

bool Spliner::checkOtSidePossible(const std::string & more_space) const
{
  if (std::abs(cur_d_) > 0.25 && more_space != last_ot_side_) {
    log(LogLevel::Info, "Can't switch sides, because we are not on the raceline");
    return false;
  }
  return true;
}

std::pair<std::string, double> Spliner::moreSpace(
  const f110_msgs::msg::Obstacle & obstacle,
  const std::vector<f110_msgs::msg::Wpnt> & gb_wpnts, const std::vector<long> & gb_idxs) const
{
  const auto & w = gb_wpnts[static_cast<std::size_t>(gb_idxs[0])];
  const double left_gap = std::abs(w.d_left - obstacle.d_left);
  const double right_gap = std::abs(w.d_right + obstacle.d_right);
  const double min_space = params_.evasion_dist + params_.spline_bound_mindist;

  if (right_gap > min_space && left_gap < min_space) {
    double d_apex_right = obstacle.d_right - params_.evasion_dist;
    if (d_apex_right > 0.0) {
      d_apex_right = 0.0;
    }
    return {"right", d_apex_right};
  }
  if (left_gap > min_space && right_gap < min_space) {
    double d_apex_left = obstacle.d_left + params_.evasion_dist;
    if (d_apex_left < 0.0) {
      d_apex_left = 0.0;
    }
    return {"left", d_apex_left};
  }
  double candidate_left = obstacle.d_left + params_.evasion_dist;
  double candidate_right = obstacle.d_right - params_.evasion_dist;
  if (std::abs(candidate_left) <= std::abs(candidate_right)) {
    if (candidate_left < 0.0) {
      candidate_left = 0.0;
    }
    return {"left", candidate_left};
  }
  if (candidate_right > 0.0) {
    candidate_right = 0.0;
  }
  return {"right", candidate_right};
}

std::pair<f110_msgs::msg::Obstacle, std::vector<f110_msgs::msg::Obstacle>>
Spliner::groupObstacles(const std::vector<f110_msgs::msg::Obstacle> & close_obs) const
{
  const double L = gb_max_s_;
  auto unwrap_span = [&](const f110_msgs::msg::Obstacle & o) {
      const double length = pyMod(o.s_end - o.s_start, L);
      // signed unwrap: more than half a lap "ahead" means behind the car
      double start = pyMod(o.s_start - cur_s_, L);
      if (start > L / 2.0) {
        start -= L;
      }
      return std::make_pair(start, start + length);
    };

  std::vector<f110_msgs::msg::Obstacle> ordered = close_obs;
  std::stable_sort(
    ordered.begin(), ordered.end(),
    [&](const auto & a, const auto & b) {
      return pyMod(a.s_center - cur_s_, L) < pyMod(b.s_center - cur_s_, L);
    });

  std::size_t chain_len = 1;
  double chain_end = unwrap_span(ordered[0]).second;
  for (std::size_t i = 1; i < ordered.size(); ++i) {
    const auto [start, end] = unwrap_span(ordered[i]);
    if (start - chain_end < params_.obs_group_gap_m) {
      chain_len += 1;
      chain_end = std::max(chain_end, end);
    } else {
      break;
    }
  }

  std::vector<f110_msgs::msg::Obstacle> others(ordered.begin() + chain_len, ordered.end());
  if (chain_len == 1U) {
    return {ordered[0], others};
  }

  f110_msgs::msg::Obstacle merged;
  const auto & first = ordered[0];
  const auto & last = ordered[chain_len - 1];
  merged.id = first.id;
  merged.s_start = first.s_start;
  merged.s_end = last.s_end;
  merged.s_center = pyMod(merged.s_start + pyMod(merged.s_end - merged.s_start, L) / 2.0, L);
  merged.d_left = first.d_left;
  merged.d_right = first.d_right;
  bool all_static = true;
  bool any_visible = false;
  for (std::size_t i = 0; i < chain_len; ++i) {
    merged.d_left = std::max(merged.d_left, ordered[i].d_left);
    merged.d_right = std::min(merged.d_right, ordered[i].d_right);
    all_static = all_static && ordered[i].is_static;
    any_visible = any_visible || ordered[i].is_visible;
  }
  merged.d_center = (merged.d_left + merged.d_right) / 2.0;
  merged.size = pyMod(merged.s_end - merged.s_start, L);
  merged.vs = first.vs;
  merged.vd = first.vd;
  merged.is_static = all_static;
  merged.is_visible = any_visible;
  log(
    LogLevel::InfoThrottled,
    "Grouped " + std::to_string(chain_len) + " consecutive obstacles into one: " +
    format("s=[%.2f,%.2f] d=[%.2f,%.2f]", merged.s_start, merged.s_end, merged.d_right,
    merged.d_left));
  return {merged, others};
}

bool Spliner::lineHitsObstacle(
  const std::vector<double> & evasion_s, const std::vector<double> & evasion_d,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  const double L = gb_max_s_;
  const double ev = params_.evasion_dist;
  for (const auto & obs : obstacles) {
    const double span = pyMod(obs.s_end - obs.s_start, L) + 2.0 * ev;
    bool hit = false;
    for (std::size_t i = 0; i < evasion_s.size(); ++i) {
      const bool in_span = pyMod(evasion_s[i] - (obs.s_start - ev), L) <= span;
      if (in_span && evasion_d[i] > obs.d_right - ev && evasion_d[i] < obs.d_left + ev) {
        hit = true;
        break;
      }
    }
    if (hit) {
      log(
        LogLevel::Warn,
        format(
          "Evasion line passes through obstacle at s=[%.2f,%.2f], aborting evasion",
          obs.s_start, obs.s_end));
      return true;
    }
  }
  return false;
}

SplinerOutput Spliner::doSpline(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<f110_msgs::msg::Wpnt> & gb_wpnts, bool viz_on, bool want_propagated)
{
  SplinerOutput out;
  const double L = gb_max_s_;
  if (gb_wpnts.size() < 2U) {
    return out;
  }
  const double wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m;

  const auto close_obs = obsFiltering(obstacles, out, want_propagated);
  if (close_obs.empty()) {
    return out;
  }
  out.evaluated = true;

  const auto [closest_obs, other_obs] = groupObstacles(close_obs);

  double s_apex;
  if (closest_obs.s_end < closest_obs.s_start) {
    s_apex = (closest_obs.s_end + L + closest_obs.s_start) / 2.0;
  } else {
    s_apex = (closest_obs.s_end + closest_obs.s_start) / 2.0;
  }
  // PY-QUIRK: python wraps the index with gb_max_idx (= last id) instead of the
  // waypoint count; kept so that the chosen waypoints are identical
  const long n_wpnts = static_cast<long>(gb_wpnts.size());
  auto wrap_idx = [&](double value) {
      long idx = static_cast<long>(value);  // python int(): truncation
      idx = idx % gb_max_idx_;
      if (idx < 0) {
        idx += gb_max_idx_;
      }
      return std::min(idx, n_wpnts - 1);  // safety clamp (python would IndexError)
    };
  // evasion cache (opt-in): a static obstacle (vs = vd = 0) has tick-invariant geometry; dynamic ones bypass it
  const bool cacheable = params_.cache_enable && closest_obs.is_static &&
    closest_obs.vs == 0.0 && closest_obs.vd == 0.0;
  EvasionCacheEntry * cached = cacheable ? findCacheEntry(closest_obs) : nullptr;

  std::string more_space;
  std::string outside;
  double d_apex = 0.0;
  double considered_x = 0.0;
  double considered_y = 0.0;
  bool bound_danger = false;
  if (cached != nullptr) {
    ++cache_hits_;
    cached->last_use = ++cache_tick_;
    more_space = cached->more_space;
    outside = cached->outside;
    d_apex = cached->d_apex;
    considered_x = cached->considered_x;
    considered_y = cached->considered_y;
    bound_danger = cached->bound_danger;
    out.wpnts = cached->wpnts;
    if (viz_on) {
      out.has_considered = true;
      out.considered_x = considered_x;
      out.considered_y = considered_y;
      for (const auto & w : out.wpnts) {
        out.markers.push_back({w.x_m, w.y_m, w.vx_mps});
      }
    }
    log(
      LogLevel::InfoThrottled,
      "Spliner cache HIT obs id " + std::to_string(cached->obs_id) + " (" +
      std::to_string(cache_.size()) + " entries, " + std::to_string(cache_hits_) + " hits / " +
      std::to_string(cache_misses_) + " misses)");
  } else {
    if (cacheable) {
      ++cache_misses_;
    }
    std::vector<long> gb_idxs;
    double kappa_sum = 0.0;
    for (int i = 0; i < 20; ++i) {
      const long idx = wrap_idx(s_apex / wpnt_dist + i);
      gb_idxs.push_back(idx);
      kappa_sum += gb_wpnts[static_cast<std::size_t>(idx)].kappa_radpm;
    }
    outside = kappa_sum < 0.0 ? "left" : "right";
    std::tie(more_space, d_apex) = moreSpace(closest_obs, gb_wpnts, gb_idxs);

    considered_x = gb_wpnts[static_cast<std::size_t>(gb_idxs[0])].x_m;
    considered_y = gb_wpnts[static_cast<std::size_t>(gb_idxs[0])].y_m;
    if (viz_on) {
      out.has_considered = true;
      out.considered_x = considered_x;
      out.considered_y = considered_y;
    }

    // scale dst linearly between 1 and 1.5 depending on the speed normalised to the max speed
    double scale = std::clamp(1.0 + cur_vs_ / gb_vmax_, 1.0, 1.5);
    if (outside == more_space) {
      scale *= 1.75;
    }
    const double obs_len = pyMod(closest_obs.s_end - closest_obs.s_start, L);
    const double s_front = s_apex - obs_len / 2.0;
    const double s_rear = s_apex + obs_len / 2.0;
    std::vector<std::pair<double, double>> evasion_points;
    for (const double dst : {params_.pre_apex_0, params_.pre_apex_1, params_.pre_apex_2}) {
      evasion_points.emplace_back(s_front + dst * scale, 0.0);
    }
    evasion_points.emplace_back(s_front, d_apex);
    evasion_points.emplace_back(s_apex, d_apex);
    evasion_points.emplace_back(s_rear, d_apex);
    for (const double dst : {params_.post_apex_0, params_.post_apex_1, params_.post_apex_2}) {
      evasion_points.emplace_back(s_rear + dst * scale, 0.0);
    }
    // drop non-increasing knots (post apexes at 0 collapse onto the rear knot)
    std::vector<std::pair<double, double>> unique_points{evasion_points[0]};
    for (std::size_t i = 1; i < evasion_points.size(); ++i) {
      if (evasion_points[i].first > unique_points.back().first + 1e-6) {
        unique_points.push_back(evasion_points[i]);
      }
    }
    double plateau_end = s_rear;
    if (unique_points.back().first <= s_rear + 1e-6) {
      plateau_end = s_rear + params_.post_apex_hold_m;
      unique_points.emplace_back(plateau_end, d_apex);
    }

    // Spline spatially for d with s as base (scipy InterpolatedUnivariateSpline k=3)
    const double spline_resolution = 0.1;
    std::vector<double> knots_s, knots_d;
    for (const auto & p : unique_points) {
      knots_s.push_back(p.first);
      knots_d.push_back(p.second);
    }
    if (knots_s.size() < 4U) {
      // python: scipy raises (m > k required) and the timer callback dies;
      // degrade to the lower-order scipy.CubicSpline behaviour instead
      log(LogLevel::Warn, "Fewer than 4 spline knots, using lower-order spline");
    }
    if (knots_s.size() < 2U) {
      return out;
    }
    const CubicSpline spatial_spline(knots_s, knots_d);

    // numpy.arange(start, stop, step): n = ceil((stop-start)/step), values start + i*delta
    const double start = knots_s.front();
    const double stop = knots_s.back();
    const long n_points = static_cast<long>(std::ceil((stop - start) / spline_resolution));
    const double delta = (start + spline_resolution) - start;
    std::vector<double> evasion_s(static_cast<std::size_t>(std::max(0L, n_points)));
    std::vector<double> evasion_d(evasion_s.size());
    for (std::size_t i = 0; i < evasion_s.size(); ++i) {
      evasion_s[i] = start + static_cast<double>(i) * delta;
      const double v = spatial_spline(evasion_s[i]);
      evasion_d[i] = d_apex < 0.0 ? std::clamp(v, d_apex, 0.0) : std::clamp(v, 0.0, d_apex);
      // hard-guarantee full d_apex clearance alongside the obstacle body (and hold tail)
      if (evasion_s[i] >= s_front && evasion_s[i] <= plateau_end) {
        evasion_d[i] = d_apex;
      }
    }
    for (auto & s : evasion_s) {
      s = pyMod(s, L);
    }

    for (std::size_t i = 0; i < evasion_s.size(); ++i) {
      const long gb_wpnt_i = std::min(
        static_cast<long>(pyMod(evasion_s[i] / wpnt_dist, static_cast<double>(gb_max_idx_))),
        n_wpnts - 1);
      const auto & gw = gb_wpnts[static_cast<std::size_t>(gb_wpnt_i)];
      if (std::abs(evasion_d[i]) > spline_resolution) {
        const double tb_dist = more_space == "left" ? gw.d_left : gw.d_right;
        if (std::abs(evasion_d[i]) > std::abs(tb_dist) - params_.spline_bound_mindist) {
          log(LogLevel::Info, "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
          bound_danger = true;  // invariant part of the danger flag
          break;
        }
      }
      const double vi = outside == more_space ? gw.vx_mps : gw.vx_mps * 0.9;
      const auto p = converter_.getCartesian(evasion_s[i], evasion_d[i]);
      f110_msgs::msg::Wpnt wpnt;
      wpnt.id = static_cast<int>(out.wpnts.size());
      wpnt.x_m = p.x;
      wpnt.y_m = p.y;
      wpnt.s_m = evasion_s[i];
      wpnt.d_m = evasion_d[i];
      wpnt.vx_mps = vi;
      out.wpnts.push_back(wpnt);
      if (viz_on) {
        out.markers.push_back({p.x, p.y, vi});
      }
    }

    if (cacheable) {
      EvasionCacheEntry entry;
      entry.obs_id = closest_obs.id;
      entry.s_start = closest_obs.s_start;
      entry.s_end = closest_obs.s_end;
      entry.d_left = closest_obs.d_left;
      entry.d_right = closest_obs.d_right;
      entry.vs_at_compute = cur_vs_;
      if (!bound_danger) {
        entry.wpnts = out.wpnts;
      }
      entry.more_space = more_space;
      entry.outside = outside;
      entry.d_apex = d_apex;
      entry.considered_x = considered_x;
      entry.considered_y = considered_y;
      entry.bound_danger = bound_danger;
      storeCacheEntry(std::move(entry));
    }
  }

  // per-tick checks: car state (side gate) and the other obstacles change between ticks,
  // so they run on the cached and on the freshly built evasion alike
  bool danger_flag = bound_danger;
  if (!checkOtSidePossible(more_space)) {
    danger_flag = true;
  }
  if (!danger_flag) {
    std::vector<double> evasion_s, evasion_d;
    evasion_s.reserve(out.wpnts.size());
    evasion_d.reserve(out.wpnts.size());
    for (const auto & w : out.wpnts) {
      evasion_s.push_back(w.s_m);
      evasion_d.push_back(w.d_m);
    }
    if (lineHitsObstacle(evasion_s, evasion_d, other_obs)) {
      danger_flag = true;
    }
  }

  if (!danger_flag) {
    out.ot_side = more_space;
    out.ot_line = outside;
    out.side_switch = last_ot_side_ != more_space;
    if (last_ot_side_ != more_space) {
      out.switch_time_now = true;
    }
    last_ot_side_ = more_space;
  } else {
    out.danger = true;
    out.wpnts.clear();
    out.markers.clear();
    out.side_switch = true;  // this fools the statemachine to cool down
    out.switch_time_now = true;
  }
  return out;
}

}  // namespace spline_planner
