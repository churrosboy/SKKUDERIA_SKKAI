#ifndef PERCEPTION__TRACKING_CORE_HPP_
#define PERCEPTION__TRACKING_CORE_HPP_

// ROS-free C++ port of perception/tracking.py (StaticDynamic); behaviour is kept
// identical to the python node (quirks marked "PY-QUIRK") for the golden replay test.

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt.hpp>
#include <frenet_conversion/frenet_converter.hpp>

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace perception
{

/// Python `a % b` (result carries the sign of the modulus).
double pyMod(double value, double modulus);

/// tracking.py normalize_s: wrap s into (-L/2, L/2].
double normalizeS(double s, double track_length);

struct TrackingParams
{
  // static (launch-time) parameters
  double rate_hz{40.0};
  double p_vs{0.2};
  double p_d{0.02};
  double p_vd{0.2};
  double measurement_var_s{0.002};
  double measurement_var_d{0.002};
  double measurement_var_vs{0.2};
  double measurement_var_vd{0.2};
  double process_var_vs{2.0};
  double process_var_vd{8.0};
  double max_dist{0.5};
  double var_pub{1.0};
  // dynamic-reconfigurable parameters
  int ttl_dynamic{40};
  double ratio_to_glob_path{0.6};
  int ttl_static{3};
  int min_nb_meas{6};
  double min_std{0.16};
  double max_std{0.2};
  double dist_deletion{7.0};
  double dist_infront{8.0};
  double vs_reset{0.1};
  double aggro_multiplier{2.0};
  bool debug_mode{false};
  bool publish_static{true};
  bool no_memory_mode{false};
  // hard-coded in the python node
  double fov_dist_margin{0.4};
};

/// python ObstacleSD.staticFlag: None / True / False
enum class StaticFlag { Unknown, Static, Dynamic };

struct TrackedObstacle
{
  int id{0};
  std::vector<double> measurements_s;
  std::vector<double> measurements_d;
  std::array<double, 2> mean{{0.0, 0.0}};  // [mean_s, mean_d]
  int static_count{0};
  int total_count{0};
  int nb_meas{0};
  int ttl{0};
  bool is_in_front{true};
  int current_lap{0};
  StaticFlag static_flag{StaticFlag::Unknown};
  double size{0.0};
  int nb_detection{0};
  bool is_visible{true};

  TrackedObstacle(
    int id_, double s_meas, double d_meas, int lap, double size_, bool is_visible_,
    int ttl_);

  void updateMean(double track_length);
  double stdS(double track_length) const;
  double stdD() const;
  /// python ObstacleSD.isStatic
  void classifyStatic(const TrackingParams & params, double track_length);
};

/// python Opponent_state: EKF on X = [s, v_s, d, v_d], Z = [s, v_s, d, v_d]
class OpponentState
{
public:
  /// Build F, Q, R, P from the launch-time params (python __init__).
  void configure(const TrackingParams & params);

  double targetVelocity(
    const TrackingParams & params, const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    double track_length) const;
  void predict(
    const TrackingParams & params, const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    double track_length);
  void update(const TrackingParams & params, const TrackedObstacle & tracked, double track_length);
  /// python StaticDynamic.initialize_dynamic_obstacle
  void initialise(const TrackingParams & params, const TrackedObstacle & tracked);

  double meanVsFilt() const;
  double meanVdFilt() const;

  Eigen::Vector4d x{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
  int id{0};
  double size{0.0};
  bool is_initialised{false};
  std::vector<double> vs_list;
  double avg_vs{0.0};
  bool use_target_vel{false};
  int ttl{0};
  std::array<double, 5> vs_filt{};
  std::array<double, 5> vd_filt{};

private:
  Eigen::Matrix4d F_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d Q_{Eigen::Matrix4d::Zero()};
  Eigen::Matrix4d R_{Eigen::Matrix4d::Identity()};
  double rate_hz_{40.0};
};

struct ScanData
{
  std::vector<float> ranges;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
};

/// Static/dynamic obstacle tracker (python StaticDynamic minus ROS plumbing).
class Tracker
{
public:
  explicit Tracker(const TrackingParams & params);

  TrackingParams & params() {return params_;}
  const TrackingParams & params() const {return params_;}

  /// python pathCallback: accepted only once. Returns false if ignored/invalid.
  bool setGlobalPath(const std::vector<f110_msgs::msg::Wpnt> & waypoints);
  bool hasPath() const {return initialized_track_bounds_;}

  void setCarS(double s);                               // python carStateCallback
  void setCarPose(double x, double y, double yaw);       // python carStateGlobCallback
  void setScan(ScanData scan);                           // python scansCallback
  void setMeasurements(std::vector<f110_msgs::msg::Obstacle> obstacles);  // obstacleCallback

  /// python loop() without the publishing: opponent predict + update().
  void step();

  /// python publishObstacles(): fills the /perception/obstacles and
  /// /perception/raw_obstacles obstacle lists.
  void buildObstacleArrays(
    std::vector<f110_msgs::msg::Obstacle> & estimated,
    std::vector<f110_msgs::msg::Obstacle> & raw_opponent) const;

  const std::vector<TrackedObstacle> & trackedObstacles() const {return tracked_obstacles_;}
  const OpponentState & opponent() const {return opponent_;}
  double trackLength() const {return track_length_;}
  int currentLap() const {return current_lap_;}
  const frenet_conversion::FrenetConverter & converter() const {return converter_;}

  // exposed for unit tests
  bool checkInFieldOfView(
    double vec_x, double vec_y, double cos_yaw, double sin_yaw) const;
  static double angleToObstacle(double vec_x, double vec_y, double cos_yaw, double sin_yaw);

private:
  void update();
  void lapUpdate(double car_s);
  const f110_msgs::msg::Obstacle * verifyPosition(
    const TrackedObstacle & obstacle,
    const std::vector<const f110_msgs::msg::Obstacle *> & candidates) const;
  const f110_msgs::msg::Obstacle * closestCandidate(
    double max_dist, double s, double d,
    const std::vector<const f110_msgs::msg::Obstacle *> & candidates) const;
  void updateTrackedObstacle(TrackedObstacle & tracked, const f110_msgs::msg::Obstacle & meas);
  bool checkInFront(const TrackedObstacle & tracked, double car_s) const;
  double distanceObsCar(const TrackedObstacle & tracked, double car_s) const;

  TrackingParams params_;
  std::vector<f110_msgs::msg::Obstacle> meas_obstacles_;
  std::vector<TrackedObstacle> tracked_obstacles_;
  OpponentState opponent_;

  bool initialized_track_bounds_{false};
  std::vector<f110_msgs::msg::Wpnt> global_path_;
  double track_length_{-1.0};
  frenet_conversion::FrenetConverter converter_;

  std::optional<double> car_s_;
  std::optional<double> last_car_s_;
  double car_x_{0.0};
  double car_y_{0.0};
  double car_cos_yaw_{1.0};
  double car_sin_yaw_{0.0};

  ScanData scan_;
  int current_lap_{0};
  int current_id_{1};
};

}  // namespace perception

#endif  // PERCEPTION__TRACKING_CORE_HPP_
