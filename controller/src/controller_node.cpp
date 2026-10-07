// C++ port of controller/controller_manager.py -- PP mode only (MAP/FTG not ported).
// Line references are into controller_manager.py unless noted.

#include "controller/controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <yaml-cpp/yaml.h>

namespace controller
{

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

// python % semantics for the legacy obstacle pick (:628)
double pyMod(double a, double b)
{
  double r = std::fmod(a, b);
  return r < 0.0 ? r + b : r;
}

rcl_interfaces::msg::ParameterDescriptor doubleRange(
  double from, double to, double step)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = step;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

}  // namespace

ControllerNode::ControllerNode()
: Node("controller")  // launch overrides the python node's name to "controller"
                      // too; using it as the default keeps `ros2 run` identical
{
}

void ControllerNode::init()
{
  // ---- static params from launch (:91-94) ----
  const std::string mode = declare_parameter<std::string>("mode", "PP");
  const std::string lu_table = declare_parameter<std::string>("LU_table", "");
  declare_parameter<bool>("mapping", false);  // feeds the FTG fixed-speed branch
  (void)lu_table;  // PP does not use the steering LUT; declared so launch files
                   // passing LU_table don't fail

  if (mode != "PP") {
    RCLCPP_FATAL(
      get_logger(),
      "controller_cpp only implements PP mode (got mode:=%s). "
      "MAP/FTG are not ported -- relaunch with ctrl_exec:=controller (python).",
      mode.c_str());
    throw std::invalid_argument("controller_cpp: unsupported mode " + mode);
  }

  // ---- remote parameters (:81-84); map_path is fetched by python but unused ----
  racecar_version_ = getRemoteParameter("global_parameters", "racecar_version").as_string();
  const auto sim_param = getRemoteParameter("global_parameters", "sim");
  sim_ = sim_param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ?
    sim_param.as_bool() : (sim_param.as_int() != 0);
  const auto sm_rate = getRemoteParameter("state_machine", "rate_hz");
  state_machine_rate_ = sm_rate.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
    sm_rate.as_double() : static_cast<double>(sm_rate.as_int());

  const std::string stack_master_share =
    ament_index_cpp::get_package_share_directory("stack_master");
  wheelbase_ = readWheelbase(stack_master_share);
  RCLCPP_INFO(
    get_logger(), "PP controller (C++): racecar_version=%s sim=%s wheelbase=%.3f",
    racecar_version_.c_str(), sim_ ? "true" : "false", wheelbase_);

  // ---- l1 + node-level parameters ----
  declareL1Parameters();
  pp_params_ = buildPpParams();
  pp_core_.emplace(pp_params_);

  // ---- ftg fallback for the FTGONLY state (init_ftg_controller, :436-478) ----
  declareFtgParameters();
  ftg_core_.emplace(ftg_params_);

  // ---- publishers (:96-104) ----
  drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  steering_pub_ = create_publisher<visualization_msgs::msg::Marker>("steering", 10);
  lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 10);
  trailing_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "trailing_opponent_marker", 10);
  // never published (python's set_waypoint_markers is dead code); declared for
  // topic-graph parity
  waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("my_waypoints", 10);
  l1_pub_ = create_publisher<geometry_msgs::msg::Point>("l1_distance", 10);
  gap_data_pub_ = create_publisher<f110_msgs::msg::PidData>("/trailing/gap_data", 10);

  // ---- subscribers (:210-218, imu :401, scan :222) ----
  state_sub_ = create_subscription<std_msgs::msg::String>(
    "/state", 10, std::bind(&ControllerNode::stateCb, this, _1));
  gb_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", 10, std::bind(&ControllerNode::trackLengthCb, this, _1));
  obstacle_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    "/perception/obstacles", 10, std::bind(&ControllerNode::obstacleCb, this, _1));
  trailing_target_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    "/trailing_target", 10, std::bind(&ControllerNode::trailingTargetCb, this, _1));
  local_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/local_waypoints", 10, std::bind(&ControllerNode::localWaypointCb, this, _1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/odom", 10, std::bind(&ControllerNode::odomCb, this, _1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/car_state/pose", 10, std::bind(&ControllerNode::carStateCb, this, _1));
  frenet_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", 10, std::bind(&ControllerNode::carStateFrenetCb, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/vesc/sensors/imu/raw", 10, std::bind(&ControllerNode::imuCb, this, _1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),  // python: qos_profile_sensor_data (:222)
    std::bind(&ControllerNode::scanCb, this, _1));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&ControllerNode::parametersCallback, this, _1));

  // node-clock timer (matches python create_timer; honors use_sim_time if ever set)
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_),
    std::bind(&ControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Controller ready");
}

rclcpp::Parameter ControllerNode::getRemoteParameter(
  const std::string & remote_node_name, const std::string & param_name)
{
  // Port of get_remote_parameter (:478-494) with a bounded wait: python spins
  // forever, which hides a dead dependency -- here 30 s then fatal.
  auto client = std::make_shared<rclcpp::SyncParametersClient>(this, remote_node_name);
  int attempts = 0;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("interrupted while waiting for " + remote_node_name);
    }
    if (++attempts >= 30) {
      RCLCPP_FATAL(
        get_logger(), "parameter service of %s not available after 30 s",
        remote_node_name.c_str());
      throw std::runtime_error(remote_node_name + " parameter service unavailable");
    }
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }
  const auto params = client->get_parameters({param_name});
  if (params.empty() ||
    params[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    throw std::runtime_error(
            remote_node_name + " did not provide parameter " + param_name);
  }
  return params[0];
}

double ControllerNode::readWheelbase(const std::string & stack_master_share) const
{
  // Port of init_pp_controller's wheelbase block (:388-398)
  namespace fs = std::filesystem;
  if (sim_) {
    const auto path = fs::path(stack_master_share) / "config" / racecar_version_ /
      "sim_params.yaml";
    const YAML::Node car_params = YAML::LoadFile(path.string());
    return car_params["lr"].as<double>() + car_params["lf"].as<double>();
  }
  const auto path = fs::path(stack_master_share) / "config" / racecar_version_ /
    "vesc.yaml";
  const YAML::Node car_params = YAML::LoadFile(path.string());
  return car_params["vesc_to_odom_node"]["ros__parameters"]["wheelbase"].as<double>();
}

void ControllerNode::declareL1Parameters()
{
  // l1_params.yaml arrives as standard parameter overrides; keys python indexes with [] are REQUIRED (:231-303)
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  const std::vector<std::string> required = {
    "t_clip_min", "t_clip_max", "m_l1", "q_l1", "speed_lookahead",
    "lat_err_coeff", "acc_scaler_for_steer", "dec_scaler_for_steer",
    "start_scale_speed", "end_scale_speed", "downscale_factor",
    "speed_lookahead_for_steer", "prioritize_dyn", "trailing_gap",
    "trailing_p_gain", "trailing_i_gain", "trailing_d_gain",
    "blind_trailing_speed"};
  std::string missing;
  for (const auto & name : required) {
    if (overrides.find(name) == overrides.end()) {
      missing += (missing.empty() ? "" : ", ") + name;
    }
  }
  if (!missing.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "l1_params.yaml overrides missing required keys: %s -- is the "
      "<param from=.../l1_params.yaml> line present in controller_launch.xml?",
      missing.c_str());
    throw std::runtime_error("missing l1 parameters: " + missing);
  }

  // Numeric declare with dynamic typing (yaml ints stay valid); the default is a placeholder for required keys
  auto declareNumber = [this](const std::string & name, double default_value,
      rcl_interfaces::msg::ParameterDescriptor descriptor) {
      descriptor.dynamic_typing = true;
      declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
    };

  // ranges copied from param_dicts (:231-303)
  declareNumber("t_clip_min", 0.0, doubleRange(0.0, 1.5, 0.01));
  declareNumber("t_clip_max", 0.0, doubleRange(0.0, 10.0, 0.01));
  declareNumber("m_l1", 0.0, doubleRange(0.0, 1.0, 0.001));
  declareNumber("q_l1", 0.0, doubleRange(-1.0, 1.0, 0.001));
  declareNumber("speed_lookahead", 0.0, doubleRange(0.0, 1.0, 0.01));
  declareNumber("lat_err_coeff", 0.0, doubleRange(0.0, 1.0, 0.01));
  declareNumber("acc_scaler_for_steer", 0.0, doubleRange(0.0, 1.5, 0.01));
  declareNumber("dec_scaler_for_steer", 0.0, doubleRange(0.0, 1.5, 0.01));
  declareNumber("start_scale_speed", 0.0, doubleRange(0.0, 10.0, 0.01));
  declareNumber("end_scale_speed", 0.0, doubleRange(0.0, 10.0, 0.01));
  declareNumber("downscale_factor", 0.0, doubleRange(0.0, 0.5, 0.01));
  declareNumber("speed_lookahead_for_steer", 0.0, doubleRange(0.0, 0.2, 0.01));
  declare_parameter<bool>("prioritize_dyn", true);
  declareNumber("trailing_gap", 0.0, doubleRange(0.0, 3.0, 0.1));
  declareNumber("trailing_p_gain", 0.0, doubleRange(0.0, 3.0, 0.01));
  declareNumber("trailing_i_gain", 0.0, doubleRange(0.0, 0.5, 0.001));
  declareNumber("trailing_d_gain", 0.0, doubleRange(0.0, 1.0, 0.01));
  declareNumber("blind_trailing_speed", 0.0, doubleRange(0.0, 3.0, 0.01));
  // .get() keys -- python code defaults (:286-302)
  declareNumber("trailing_vel_gain", 0.0, doubleRange(0.0, 1.0, 0.01));
  declareNumber("trailing_min_speed", 0.0, doubleRange(0.0, 2.0, 0.01));
  declare_parameter<bool>("trailing_creep_always", false);
  declareNumber("trailing_stop_gap", 0.3, doubleRange(0.0, 3.0, 0.05));
  declareNumber("trailing_nose_offset", 0.45, doubleRange(0.0, 1.0, 0.01));
  declare_parameter<bool>("use_sm_trailing_target", false);

  // node-level params (declared inline in __init__, :126-179)
  const rcl_interfaces::msg::ParameterDescriptor no_range;
  declareNumber("cmd_accel_limit", 2.0, no_range);
  declareNumber("cmd_decel_limit", 8.0, no_range);
  declareNumber("launch_accel_limit", 1.5, no_range);
  declareNumber("launch_exit_speed", 1.5, no_range);
  declareNumber("launch_creep_speed", 0.8, no_range);
  declare_parameter<bool>("merge_smooth_enable", false);
  declareNumber("merge_smooth_duration_sec", 1.5, no_range);
  declareNumber("merge_accel_limit", 1.0, no_range);
  declareNumber("merge_steer_rate_limit", 0.8, no_range);
  // RECOVERY L1 extension is off by default; set recovery_l1_gain > 0 to enable
  declareNumber("recovery_l1_gain", 0.0, no_range);
  declareNumber("recovery_t_clip_max", 6.0, no_range);
  declare_parameter<bool>("l1_curv_cap_enable", false);
  declareNumber("l1_curv_cap_max_heading", 1.57, no_range);

  // snapshot members (mirrors :190-197 and the node-level get_parameter calls)
  auto num = [this](const std::string & name) {
      const auto p = get_parameter(name);
      return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(p.as_int()) : p.as_double();
    };
  prioritize_dyn_ = get_parameter("prioritize_dyn").as_bool();
  use_sm_trailing_target_ = get_parameter("use_sm_trailing_target").as_bool();
  cmd_accel_limit_ = num("cmd_accel_limit");
  cmd_decel_limit_ = num("cmd_decel_limit");
  launch_accel_limit_ = num("launch_accel_limit");
  launch_exit_speed_ = num("launch_exit_speed");
  launch_creep_speed_ = num("launch_creep_speed");
  merge_smooth_enable_ = get_parameter("merge_smooth_enable").as_bool();
  merge_smooth_duration_sec_ = num("merge_smooth_duration_sec");
  merge_accel_limit_ = num("merge_accel_limit");
  merge_steer_rate_limit_ = num("merge_steer_rate_limit");
}

void ControllerNode::declareFtgParameters()
{
  // Port of init_ftg_controller (:436-478); the ftg_* params live on this node, defaults from ftg_defaults
  auto declareNumber = [this](const std::string & name, double default_value) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;  // yaml ints stay valid
      declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
    };
  auto num = [this](const std::string & name) {
      const auto p = get_parameter(name);
      return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(p.as_int()) : p.as_double();
    };

  declare_parameter<bool>("ftg_debug", false);   // accepted for parity; the
                                                 // rviz debug markers are not ported
  declareNumber("ftg_safety_radius", 100.0);     // [beams] ~15deg edge bubble
  declareNumber("ftg_max_lidar_dist", 5.0);      // [m]
  declareNumber("ftg_max_speed", 4.0);           // [m/s] FTG is an emergency behavior
  declareNumber("ftg_range_offset", 375.0);      // scan[range_offset:-range_offset]
  declareNumber("ftg_track_width", 1.65);        // [m]

  ftg_params_.mapping = get_parameter("mapping").as_bool();
  ftg_params_.safety_radius = static_cast<int>(num("ftg_safety_radius"));
  ftg_params_.max_lidar_dist = num("ftg_max_lidar_dist");
  ftg_params_.max_speed = num("ftg_max_speed");
  ftg_params_.range_offset = static_cast<int>(num("ftg_range_offset"));
  ftg_params_.track_width = num("ftg_track_width");
  if (get_parameter("ftg_debug").as_bool()) {
    RCLCPP_WARN(
      get_logger(), "ftg_debug markers are not ported to controller_cpp");
  }
  RCLCPP_INFO(
    get_logger(),
    "FTG fallback ready: safety_radius=%d max_lidar_dist=%.2f max_speed=%.2f "
    "range_offset=%d track_width=%.2f",
    ftg_params_.safety_radius, ftg_params_.max_lidar_dist, ftg_params_.max_speed,
    ftg_params_.range_offset, ftg_params_.track_width);
}

PpParams ControllerNode::buildPpParams() const
{
  auto num = [this](const std::string & name) {
      const auto p = get_parameter(name);
      return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(p.as_int()) : p.as_double();
    };
  PpParams p;
  p.t_clip_min = num("t_clip_min");
  p.t_clip_max = num("t_clip_max");
  p.m_l1 = num("m_l1");
  p.q_l1 = num("q_l1");
  p.speed_lookahead = num("speed_lookahead");
  p.lat_err_coeff = num("lat_err_coeff");
  p.acc_scaler_for_steer = num("acc_scaler_for_steer");
  p.dec_scaler_for_steer = num("dec_scaler_for_steer");
  p.start_scale_speed = num("start_scale_speed");
  p.end_scale_speed = num("end_scale_speed");
  p.downscale_factor = num("downscale_factor");
  p.speed_lookahead_for_steer = num("speed_lookahead_for_steer");
  p.trailing_gap = num("trailing_gap");
  p.trailing_p_gain = num("trailing_p_gain");
  p.trailing_i_gain = num("trailing_i_gain");
  p.trailing_d_gain = num("trailing_d_gain");
  p.blind_trailing_speed = num("blind_trailing_speed");
  p.trailing_vel_gain = num("trailing_vel_gain");
  p.trailing_min_speed = num("trailing_min_speed");
  p.trailing_creep_always = get_parameter("trailing_creep_always").as_bool();
  p.trailing_stop_gap = num("trailing_stop_gap");
  p.trailing_nose_offset = num("trailing_nose_offset");
  p.recovery_l1_gain = num("recovery_l1_gain");
  p.recovery_t_clip_max = num("recovery_t_clip_max");
  p.l1_curv_cap_enable = get_parameter("l1_curv_cap_enable").as_bool();
  p.l1_curv_cap_max_heading = num("l1_curv_cap_max_heading");
  p.loop_rate = rate_;
  p.wheelbase = wheelbase_;
  return p;
}

rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Replaces the python l1_param_cb (:500-567); uses the INCOMING values (get_parameter still returns the old one here)
  auto num = [](const rclcpp::Parameter & p) {
      return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(p.as_int()) : p.as_double();
    };
  for (const auto & param : parameters) {
    const std::string & n = param.get_name();
    if (n == "t_clip_min") {pp_params_.t_clip_min = num(param);} else
    if (n == "t_clip_max") {pp_params_.t_clip_max = num(param);} else
    if (n == "m_l1") {pp_params_.m_l1 = num(param);} else
    if (n == "q_l1") {pp_params_.q_l1 = num(param);} else
    if (n == "speed_lookahead") {pp_params_.speed_lookahead = num(param);} else
    if (n == "lat_err_coeff") {pp_params_.lat_err_coeff = num(param);} else
    if (n == "acc_scaler_for_steer") {pp_params_.acc_scaler_for_steer = num(param);} else
    if (n == "dec_scaler_for_steer") {pp_params_.dec_scaler_for_steer = num(param);} else
    if (n == "start_scale_speed") {pp_params_.start_scale_speed = num(param);} else
    if (n == "end_scale_speed") {pp_params_.end_scale_speed = num(param);} else
    if (n == "downscale_factor") {pp_params_.downscale_factor = num(param);} else
    if (n == "speed_lookahead_for_steer") {
      pp_params_.speed_lookahead_for_steer = num(param);
    } else
    if (n == "trailing_gap") {pp_params_.trailing_gap = num(param);} else
    if (n == "trailing_p_gain") {pp_params_.trailing_p_gain = num(param);} else
    if (n == "trailing_i_gain") {pp_params_.trailing_i_gain = num(param);} else
    if (n == "trailing_d_gain") {pp_params_.trailing_d_gain = num(param);} else
    if (n == "blind_trailing_speed") {pp_params_.blind_trailing_speed = num(param);} else
    if (n == "trailing_vel_gain") {pp_params_.trailing_vel_gain = num(param);} else
    if (n == "trailing_min_speed") {pp_params_.trailing_min_speed = num(param);} else
    if (n == "trailing_creep_always") {
      pp_params_.trailing_creep_always = param.as_bool();
    } else
    if (n == "trailing_stop_gap") {pp_params_.trailing_stop_gap = num(param);} else
    if (n == "trailing_nose_offset") {pp_params_.trailing_nose_offset = num(param);} else
    if (n == "recovery_l1_gain") {pp_params_.recovery_l1_gain = num(param);} else
    if (n == "recovery_t_clip_max") {pp_params_.recovery_t_clip_max = num(param);} else
    if (n == "l1_curv_cap_enable") {
      pp_params_.l1_curv_cap_enable = param.as_bool();
    } else
    if (n == "l1_curv_cap_max_heading") {
      pp_params_.l1_curv_cap_max_heading = num(param);
    } else
    if (n == "use_sm_trailing_target") {use_sm_trailing_target_ = param.as_bool();} else
    if (n == "cmd_accel_limit") {cmd_accel_limit_ = num(param);} else
    if (n == "cmd_decel_limit") {cmd_decel_limit_ = num(param);} else
    if (n == "launch_accel_limit") {launch_accel_limit_ = num(param);} else
    if (n == "launch_exit_speed") {launch_exit_speed_ = num(param);} else
    if (n == "launch_creep_speed") {launch_creep_speed_ = num(param);} else
    if (n == "merge_smooth_enable") {merge_smooth_enable_ = param.as_bool();} else
    if (n == "merge_smooth_duration_sec") {merge_smooth_duration_sec_ = num(param);} else
    if (n == "merge_accel_limit") {merge_accel_limit_ = num(param);} else
    if (n == "merge_steer_rate_limit") {merge_steer_rate_limit_ = num(param);} else
    if (n == "ftg_safety_radius") {
      ftg_params_.safety_radius = static_cast<int>(num(param));
    } else
    if (n == "ftg_max_lidar_dist") {ftg_params_.max_lidar_dist = num(param);} else
    if (n == "ftg_max_speed") {ftg_params_.max_speed = num(param);} else
    if (n == "ftg_range_offset") {
      ftg_params_.range_offset = static_cast<int>(num(param));
    } else
    if (n == "ftg_track_width") {ftg_params_.track_width = num(param);}
    // prioritize_dyn is intentionally NOT applied live (python parity, see :618)
  }
  if (pp_core_.has_value()) {
    pp_core_->setParams(pp_params_);
  }
  if (ftg_core_.has_value()) {
    ftg_core_->setParams(ftg_params_);  // mirrors l1_param_cb's update_params (:563)
  }
  RCLCPP_INFO(get_logger(), "Updated parameters");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

/////////////
// CALLBACKS
/////////////

void ControllerNode::stateCb(const std_msgs::msg::String::SharedPtr msg)
{
  // merge-window edge detector (:693-708); compare BEFORE overwriting state_.
  // Startup-safe: state_ inits to "GB_TRACK" WITHOUT the "StateType." prefix.
  if (msg->data == "StateType.GB_TRACK" &&
    state_.rfind("StateType.", 0) == 0 &&
    state_ != "StateType.GB_TRACK")
  {
    if (merge_smooth_enable_) {
      merge_window_until_ = now() +
        rclcpp::Duration::from_seconds(merge_smooth_duration_sec_);
      RCLCPP_INFO(
        get_logger(), "[merge-smooth] window armed (%s -> GB_TRACK)", state_.c_str());
    }
  } else if (msg->data != "StateType.GB_TRACK") {
    merge_window_until_.reset();  // any other state cancels the window
  }
  state_ = msg->data;
}

void ControllerNode::trackLengthCb(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  if (msg->wpnts.empty()) {
    return;
  }
  track_length_ = msg->wpnts.back().s_m;
  // converter only feeds the opponent visualization marker (:574, :224)
  try {
    converter_.setGlobalTrajectory(msg->wpnts, true);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid global path: %s", error.what());
  }
}

void ControllerNode::trailingTargetCb(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  // freshness judged on receive time, not header stamp (:576-580)
  trailing_target_msg_ = msg;
  trailing_target_rx_time_ = now();
}

void ControllerNode::applySmTrailingTarget()
{
  // Port of _apply_sm_trailing_target (:582-612)
  if (!use_sm_trailing_target_) {
    return;
  }
  if (!trailing_target_msg_ || !trailing_target_rx_time_.has_value()) {
    return;  // topic never arrived: legacy fallback
  }
  const double age = (now() - *trailing_target_rx_time_).seconds();
  if (age > 0.5) {
    return;  // state machine stale/dead: legacy fallback (never stop on this)
  }
  if (trailing_target_msg_->obstacles.empty()) {
    opponent_.reset();
    sm_target_id_.reset();
    return;
  }
  const auto & ob = trailing_target_msg_->obstacles[0];
  if (sm_target_id_.has_value() && ob.id != *sm_target_id_) {
    // target identity switched: reset the gap-PID integrator (:602-608)
    pp_core_->resetTrailingIntegrator();
  }
  sm_target_id_ = ob.id;
  opponent_ = Opponent{ob.s_center, ob.d_center, ob.vs, ob.is_static,
    ob.is_visible, ob.size};
}

void ControllerNode::obstacleCb(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  // Legacy fallback pick (:614-645); only in charge when the SM target is
  // off/stale (applySmTrailingTarget overwrites opponent_ each cycle otherwise)
  if (!msg->obstacles.empty() && position_in_map_frenet_.has_value() &&
    track_length_.has_value())
  {
    bool static_flag = false;  // dynamic preferred over static when prioritize_dyn
    double closest_opp = *track_length_;
    for (const auto & obstacle : msg->obstacles) {
      const double opponent_dist =
        pyMod(obstacle.s_start - (*position_in_map_frenet_)[0], *track_length_);
      if (opponent_dist < closest_opp || (static_flag && !obstacle.is_static)) {
        closest_opp = opponent_dist;
        opponent_ = Opponent{obstacle.s_center, obstacle.d_center, obstacle.vs,
          obstacle.is_static, obstacle.is_visible, obstacle.size};
        static_flag = obstacle.is_static ? prioritize_dyn_ : false;
      }
    }
  } else {
    opponent_.reset();
  }
}

void ControllerNode::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  speed_now_ = msg->twist.twist.linear.x;
  // FTG scales its gap-detection radius with the current speed (:653-654)
  if (ftg_core_.has_value()) {
    ftg_core_->setVelocity(*speed_now_);
  }
}

void ControllerNode::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_msg_ = msg;
}

void ControllerNode::carStateCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & q = msg->pose.orientation;
  // yaw from quaternion (python: scipy Rotation -> euler zyx's z)
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  position_in_map_ = std::array<double, 3>{msg->pose.position.x, msg->pose.position.y, yaw};
}

void ControllerNode::localWaypointCb(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  // (:661-676) build [x, y, v, norm_tb, s, kappa, psi, ax] rows
  std::vector<LocalWaypoint> wpts;
  wpts.reserve(msg->wpnts.size());
  for (const auto & wp : msg->wpnts) {
    LocalWaypoint row;
    row.x = wp.x_m;
    row.y = wp.y_m;
    row.v = wp.vx_mps;
    row.norm_tb = (wp.d_right + wp.d_left != 0.0) ?
      std::min(wp.d_left, wp.d_right) / (wp.d_right + wp.d_left) : 0.0;
    row.s = wp.s_m;
    row.kappa = wp.kappa_radpm;
    row.psi = wp.psi_rad;
    row.ax = wp.ax_mps2;
    wpts.push_back(row);
  }
  waypoint_array_in_map_ = std::move(wpts);
  have_local_waypoints_ = !waypoint_array_in_map_.empty();
  waypoint_safety_counter_ = 0;
}

void ControllerNode::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // rolling buffer (:678-681); vesc imu is rotated 90 deg -> -acc_y == long acc
  for (std::size_t i = acc_now_.size() - 1; i > 0; --i) {
    acc_now_[i] = acc_now_[i - 1];
  }
  acc_now_[0] = -msg->linear_acceleration.y;
}

void ControllerNode::carStateFrenetCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  position_in_map_frenet_ = std::array<double, 4>{
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->twist.twist.linear.x, msg->twist.twist.linear.y};
}

/////////////
// MAIN LOOP
/////////////

void ControllerNode::controlLoop()
{
  // python blocks in wait_for_messages() at startup (:312-330); here the loop
  // simply refuses to command until everything needed has arrived once
  if (!track_length_.has_value() || !have_local_waypoints_ ||
    !speed_now_.has_value() || !position_in_map_.has_value() ||
    !position_in_map_frenet_.has_value())
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Controller waiting for messages (track/waypoints/odom/pose/frenet)...");
    return;
  }

  applySmTrailingTarget();  // SM target wins over obstacle_cb pick (:818)

  double speed = 0.0;
  double steer = 0.0;
  if (state_ == "StateType.COLLISION") {
    // collision_stop owns the car via the mux; hold a zeroed /drive (:819-820)
    speed = 0.0;
    steer = 0.0;
  } else if (state_ == "StateType.FTGONLY" && scan_msg_ != nullptr) {
    std::tie(speed, steer) = ftgCycle();
  } else {
    // python parity (:825-829): FTGONLY with no scan yet falls through to PP
    if (state_ == "StateType.FTGONLY") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FTGONLY but no /scan received yet -- running PP cycle (python parity)");
    }
    std::tie(speed, steer) = ppCycle();
  }

  // ---- command slew limiter + launch ramp + merge window (:830-873) ----
  const double dt = 1.0 / rate_;
  const bool merge_active = merge_smooth_enable_ &&
    merge_window_until_.has_value() &&
    now() < *merge_window_until_ &&
    state_ == "StateType.GB_TRACK";
  if (merge_window_until_.has_value() && !merge_active) {
    merge_window_until_.reset();  // expire/disarm cleanly
  }

  // launch-only ramp: arm at standstill, pin the ramp base to measured speed
  // plus creep headroom; disarm with hysteresis once properly rolling
  const double measured_speed = speed_now_.has_value() ?
    std::max(*speed_now_, 0.0) : 0.0;
  if (measured_speed < launch_standstill_speed_) {
    launch_active_ = true;
    last_cmd_speed_ = std::min(last_cmd_speed_, measured_speed + launch_creep_speed_);
  } else if (measured_speed > launch_exit_speed_) {
    launch_active_ = false;
  }

  double accel_lim = cmd_accel_limit_;
  if (launch_active_) {
    accel_lim = std::min(accel_lim, launch_accel_limit_);
  }
  if (merge_active) {
    // min() composes with the launch ramp; only accel tightened, decel free
    accel_lim = std::min(accel_lim, merge_accel_limit_);
  }
  const double decel_lim = cmd_decel_limit_;
  speed = PpCore::npClip(
    speed, last_cmd_speed_ - decel_lim * dt, last_cmd_speed_ + accel_lim * dt);
  last_cmd_speed_ = speed;

  // merge window: rate-limit steering (no absolute clamp)
  if (merge_active) {
    steer = PpCore::npClip(
      steer, last_cmd_steer_ - merge_steer_rate_limit_ * dt,
      last_cmd_steer_ + merge_steer_rate_limit_ * dt);
  }
  last_cmd_steer_ = steer;  // every tick, all branches -> slew base stays fresh

  ackermann_msgs::msg::AckermannDriveStamped ack_msg;
  ack_msg.header.stamp = now();
  ack_msg.header.frame_id = "base_link";
  ack_msg.drive.steering_angle = steer;
  ack_msg.drive.speed = speed;
  drive_pub_->publish(ack_msg);
}

std::pair<double, double> ControllerNode::ftgCycle()
{
  // Port of ftg_cycle (:797-800)
  return ftg_core_->processLidar(scan_msg_->ranges);
}

std::pair<double, double> ControllerNode::ppCycle()
{
  // Port of pp_cycle (:751-791). The recovery/l1_curv_cap params python
  // injects per cycle are already inside pp_params_ (parametersCallback).
  PpInput in;
  in.is_trailing = state_ == "StateType.TRAILING";
  in.is_recovery = state_ == "StateType.RECOVERY";
  in.x = (*position_in_map_)[0];
  in.y = (*position_in_map_)[1];
  in.yaw = (*position_in_map_)[2];
  in.s = (*position_in_map_frenet_)[0];
  in.d = (*position_in_map_frenet_)[1];
  in.vs = (*position_in_map_frenet_)[2];
  in.vd = (*position_in_map_frenet_)[3];
  in.speed_now = *speed_now_;
  in.acc_mean = std::accumulate(acc_now_.begin(), acc_now_.end(), 0.0) /
    static_cast<double>(acc_now_.size());
  in.track_length = *track_length_;
  in.opponent = opponent_;
  in.waypoints = &waypoint_array_in_map_;

  const PpOutput out = pp_core_->mainLoop(in);

  setLookaheadMarker(out.l1_x, out.l1_y, 100);
  visualizeSteering(out.steering_angle);
  visualizeTrailingOpponent();
  geometry_msgs::msg::Point l1_msg;
  l1_msg.x = static_cast<double>(out.idx_nearest_waypoint);
  l1_msg.y = out.l1_distance;
  l1_pub_->publish(l1_msg);

  double speed = out.speed;
  double steer = out.steering_angle;
  waypoint_safety_counter_ += 1;
  // same waypoints usable for 5 cycles (rate/state_machine_rate * 10, :772-776)
  if (waypoint_safety_counter_ >= rate_ / state_machine_rate_ * 10.0) {
    speed = 0.0;
    steer = 0.0;
  }

  // PID telemetry: trailing_controller() ran this very cycle, values fresh
  if (state_ == "StateType.TRAILING" && opponent_.has_value()) {
    publishPidData(out);
  }
  return {speed, steer};
}

void ControllerNode::publishPidData(const PpOutput & out)
{
  f110_msgs::msg::PidData pid_msg;
  pid_msg.header.stamp = now();
  pid_msg.should = out.gap_should;
  pid_msg.actual = out.gap;
  pid_msg.error = out.gap_error;
  pid_msg.d_value = out.v_diff;
  pid_msg.i_value = out.i_gap;
  pid_msg.input = out.trailing_command;
  gap_data_pub_->publish(pid_msg);
}

//////////////////
// VISUALIZATION
//////////////////

void ControllerNode::visualizeSteering(double theta)
{
  if (steering_pub_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "car_state/base_link";
  marker.header.stamp = now();
  marker.type = 0;  // ARROW
  marker.id = 50;
  marker.scale.x = 0.6;
  marker.scale.y = 0.05;
  marker.scale.z = 0.01;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.z = std::sin(theta / 2.0);
  marker.pose.orientation.w = std::cos(theta / 2.0);
  steering_pub_->publish(marker);
}

void ControllerNode::setLookaheadMarker(double x, double y, int id)
{
  if (lookahead_pub_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.type = 2;  // SPHERE
  marker.id = id;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1.0;
  lookahead_pub_->publish(marker);
}

void ControllerNode::visualizeTrailingOpponent()
{
  if (trailing_pub_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  const bool on = state_ == "StateType.TRAILING" && opponent_.has_value();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.type = 2;  // SPHERE
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  if (opponent_.has_value() && converter_.hasTrajectory()) {
    const auto pos = converter_.getGlobalPoint(opponent_->s_center, opponent_->d_center);
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
  }
  marker.pose.orientation.w = 1.0;
  if (!on) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  trailing_pub_->publish(marker);
}

}  // namespace controller
