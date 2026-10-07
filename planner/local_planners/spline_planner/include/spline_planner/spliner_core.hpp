#ifndef SPLINE_PLANNER__SPLINER_CORE_HPP_
#define SPLINE_PLANNER__SPLINER_CORE_HPP_

// ROS-free C++ port of spline_planner/spline_planner.py (ObstacleSpliner); kept python quirks are marked PY-QUIRK.

#include <spline_planner/py_frenet_converter.hpp>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt.hpp>

#include <functional>
#include <string>
#include <vector>

namespace spline_planner
{

double pyMod(double value, double modulus);

struct SplinerParams
{
  // stored like the python attributes: pre apexes NEGATIVE, post apexes positive
  double pre_apex_0{-3.0};
  double pre_apex_1{-2.0};
  double pre_apex_2{-1.0};
  double post_apex_0{1.0};
  double post_apex_1{1.5};
  double post_apex_2{0.0};
  double post_apex_hold_m{1.0};
  double obs_group_gap_m{3.0};
  double evasion_dist{0.4};
  double obs_traj_tresh{0.7};
  double spline_bound_mindist{0.2};
  double fixed_pred_time{0.15};
  double lookahead{10.0};  // hard-coded in python
  // evasion cache (opt-in): reuse the spline of a STATIC obstacle at a previously splined position and speed
  bool cache_enable{false};
  double cache_pos_tol_m{0.10};
  double cache_vs_tol_mps{1.0};
  int cache_max_entries{16};
};

enum class LogLevel { Info, InfoThrottled, Warn };  // *Throttled = python throttle_duration_sec=1.0
using Logger = std::function<void (LogLevel, const std::string &)>;

struct MarkerPoint
{
  double x;
  double y;
  double v;
};

struct SplinerOutput
{
  std::vector<f110_msgs::msg::Wpnt> wpnts;
  /// python: close obstacle existed -> header/flags of the OTWpntArray were filled
  bool evaluated{false};
  bool danger{false};
  std::string ot_side;
  std::string ot_line;
  bool side_switch{false};
  /// python updated last_switch_time = now (side changed, or danger)
  bool switch_time_now{false};
  /// rviz: considered obstacle (apex waypoint) and propagated obstacles
  bool has_considered{false};
  double considered_x{0.0};
  double considered_y{0.0};
  std::vector<MarkerPoint> propagated;
  std::vector<MarkerPoint> markers;
  /// no obstacles at all -> python publishes a DELETEALL marker
  bool delete_markers{false};
};

/// One memoised evasion for one (grouped) static obstacle, see SplinerParams::cache_enable.
/// Everything that depends only on (obstacle geometry, cur_vs scale, params, scaled path)
/// lives here; per-tick checks (cur_d side gate, other obstacles, side hysteresis) do NOT.
struct EvasionCacheEntry
{
  // key: merged obstacle geometry after propagation + speed the knots were scaled with
  int obs_id{0};  // informational only (tracker ids change on re-detection)
  double s_start{0.0};
  double s_end{0.0};
  double d_left{0.0};
  double d_right{0.0};
  double vs_at_compute{0.0};
  // value
  std::vector<f110_msgs::msg::Wpnt> wpnts;  // empty when bound_danger
  std::string more_space;
  std::string outside;
  double d_apex{0.0};
  double considered_x{0.0};
  double considered_y{0.0};
  bool bound_danger{false};  // trackbound veto result (invariant for this key)
  unsigned long last_use{0};  // LRU eviction
};

class Spliner
{
public:
  explicit Spliner(const SplinerParams & params, Logger logger = nullptr);

  SplinerParams & params() {return params_;}
  const SplinerParams & params() const {return params_;}

  void setGlobalPath(const std::vector<f110_msgs::msg::Wpnt> & wpnts);
  void setScaledPath(const std::vector<f110_msgs::msg::Wpnt> & wpnts);
  void setState(double s, double d, double vs);
  void setObstacles(std::vector<f110_msgs::msg::Obstacle> obstacles);
  bool hasState() const {return has_state_;}
  bool hasGlobalPath() const {return has_global_;}
  bool hasScaledPath() const {return has_scaled_;}
  bool ready() const {return has_state_ && has_global_ && has_scaled_;}

  /// python spliner_loop() minus publishing
  SplinerOutput step(bool viz_on, bool want_propagated);

  const std::string & lastOtSide() const {return last_ot_side_;}
  /// evasion cache: cleared on any param / scaled path change
  void clearCache();
  std::size_t cacheSize() const {return cache_.size();}
  unsigned cacheHits() const {return cache_hits_;}
  unsigned cacheMisses() const {return cache_misses_;}
  double gbMaxS() const {return gb_max_s_;}
  double gbVmax() const {return gb_vmax_;}
  const PyFrenetConverter & converter() const {return converter_;}

private:
  void log(LogLevel level, const std::string & message) const;
  SplinerOutput doSpline(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<f110_msgs::msg::Wpnt> & gb_wpnts, bool viz_on, bool want_propagated);
  std::vector<f110_msgs::msg::Obstacle> obsFiltering(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles, SplinerOutput & out,
    bool want_propagated) const;
  f110_msgs::msg::Obstacle predictObsMovement(
    f110_msgs::msg::Obstacle obs, SplinerOutput & out, bool want_propagated) const;
  bool checkOtSidePossible(const std::string & more_space) const;
  std::pair<std::string, double> moreSpace(
    const f110_msgs::msg::Obstacle & obstacle,
    const std::vector<f110_msgs::msg::Wpnt> & gb_wpnts, const std::vector<long> & gb_idxs) const;
  std::pair<f110_msgs::msg::Obstacle, std::vector<f110_msgs::msg::Obstacle>> groupObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & close_obs) const;
  bool lineHitsObstacle(
    const std::vector<double> & evasion_s, const std::vector<double> & evasion_d,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool cachePositionMatches(
    const EvasionCacheEntry & entry, const f110_msgs::msg::Obstacle & merged) const;
  EvasionCacheEntry * findCacheEntry(const f110_msgs::msg::Obstacle & merged);
  void storeCacheEntry(EvasionCacheEntry entry);

  SplinerParams params_;
  Logger logger_;

  std::vector<f110_msgs::msg::Obstacle> obs_;
  std::vector<f110_msgs::msg::Wpnt> gb_scaled_wpnts_;
  bool has_global_{false};
  bool has_scaled_{false};
  bool has_state_{false};
  double gb_vmax_{0.0};
  long gb_max_idx_{0};
  double gb_max_s_{0.0};
  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  std::string last_ot_side_;
  PyFrenetConverter converter_;
  // evasion cache, empty unless params_.cache_enable
  std::vector<EvasionCacheEntry> cache_;
  unsigned long cache_tick_{0};
  unsigned cache_hits_{0};
  unsigned cache_misses_{0};
};

}  // namespace spline_planner

#endif  // SPLINE_PLANNER__SPLINER_CORE_HPP_
