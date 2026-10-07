#ifndef CONTROLLER__CONTROLLER_NODE_HPP_
#define CONTROLLER__CONTROLLER_NODE_HPP_

// C++ port of controller/controller_manager.py, PP mode only (MAP/FTG are not
// ported; the FTGONLY state runs the ported ftg_core fallback on /scan).

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/pid_data.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <frenet_conversion/frenet_converter.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "controller/ftg_core.hpp"
#include "controller/pp_core.hpp"

namespace controller
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

  // Blocking startup work (remote params, yaml reads, validation) kept out of
  // the constructor; call before spinning. Throws on fatal misconfiguration.
  void init();

private:
  // startup helpers
  rclcpp::Parameter getRemoteParameter(
    const std::string & remote_node_name, const std::string & param_name);
  double readWheelbase(const std::string & stack_master_share) const;
  void declareL1Parameters();
  void declareFtgParameters();
  PpParams buildPpParams() const;

  // callbacks (port of controller_manager.py callbacks)
  void stateCb(const std_msgs::msg::String::SharedPtr msg);
  void trackLengthCb(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void obstacleCb(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void trailingTargetCb(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void localWaypointCb(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void carStateCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void carStateFrenetCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // main loop
  void controlLoop();
  void applySmTrailingTarget();
  std::pair<double, double> ppCycle();
  std::pair<double, double> ftgCycle();

  // visualization / telemetry (port of the MSG CREATION block)
  void visualizeSteering(double theta);
  void setLookaheadMarker(double x, double y, int id);
  void visualizeTrailingOpponent();
  void publishPidData(const PpOutput & out);

  // pubs
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trailing_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l1_pub_;
  rclcpp::Publisher<f110_msgs::msg::PidData>::SharedPtr gap_data_pub_;

  // subs
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr gb_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr trailing_target_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_wpnts_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // remote / startup config
  std::string racecar_version_;
  bool sim_{false};
  double state_machine_rate_{40.0};
  double wheelbase_{0.0};
  double rate_{20.0};  // controller_manager.py self.rate = 20

  // pp core + staged params (rebuilt by parametersCallback)
  std::optional<PpCore> pp_core_;
  PpParams pp_params_;

  // ftg fallback core + staged params (rebuilt by parametersCallback)
  std::optional<FtgCore> ftg_core_;
  FtgParams ftg_params_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;

  // node-level live params mirrored as members (updated in parametersCallback;
  // avoids get_parameter() calls at 20 Hz)
  double cmd_accel_limit_{2.0};
  double cmd_decel_limit_{8.0};
  double launch_accel_limit_{1.5};
  double launch_exit_speed_{1.5};
  double launch_creep_speed_{0.8};
  bool merge_smooth_enable_{false};
  double merge_smooth_duration_sec_{1.5};
  double merge_accel_limit_{1.0};
  double merge_steer_rate_limit_{0.8};
  bool use_sm_trailing_target_{false};
  bool prioritize_dyn_{true};  // init-time snapshot; live-set is a no-op for the
                               // legacy obstacle_cb pick, same as python (:618)

  // state (mirrors controller_manager.py attributes)
  std::string state_{"GB_TRACK"};  // no "StateType." prefix on purpose (:697-698)
  std::optional<double> track_length_;
  std::optional<Opponent> opponent_;
  f110_msgs::msg::ObstacleArray::SharedPtr trailing_target_msg_;
  std::optional<rclcpp::Time> trailing_target_rx_time_;
  std::optional<int32_t> sm_target_id_;
  std::vector<LocalWaypoint> waypoint_array_in_map_;
  bool have_local_waypoints_{false};
  std::optional<double> speed_now_;
  std::optional<std::array<double, 3>> position_in_map_;       // x, y, theta
  std::optional<std::array<double, 4>> position_in_map_frenet_;  // s, d, vs, vd
  int waypoint_safety_counter_{0};
  std::array<double, 10> acc_now_{};  // rolling imu buffer, mean feeds acc_scaling

  double last_cmd_speed_{0.0};
  double last_cmd_steer_{0.0};
  const double launch_standstill_speed_{0.3};  // hardcoded in python too
  bool launch_active_{true};
  std::optional<rclcpp::Time> merge_window_until_;

  frenet_conversion::FrenetConverter converter_;  // opponent marker only (:979)
};

}  // namespace controller

#endif  // CONTROLLER__CONTROLLER_NODE_HPP_
