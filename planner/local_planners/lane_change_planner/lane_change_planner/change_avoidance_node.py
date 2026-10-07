#!/usr/bin/env python3
import time
from copy import deepcopy
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from f110_msgs.msg import (
    Wpnt,
    WpntArray,
    Obstacle,
    ObstacleArray,
    OTWpntArray,
)
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Float32, Header, Bool

from frenet_conversion.frenet_converter import FrenetConverter

from lane_change_planner.ccma import CCMA
# NumPy 2.0 removed the np.row_stack alias, but the ccma library still calls it.
if not hasattr(np, "row_stack"):
    np.row_stack = np.vstack
import trajectory_planning_helpers as tph
from lane_change_planner.grid_filter import GridFilter
import matplotlib.pyplot as plt


class ChangeAvoidanceNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('change_avoidance_node')

        # Params
        self.local_wpnts = None
        self.lookahead = 8.0

        # Side hysteresis: only switch preferred_side after the new side persists this many
        # consecutive loops (20 Hz), to reject brief gap-driven side flips that jitter the path.
        self.committed_side = None
        self.pending_side = None
        self.pending_count = 0
        self.side_switch_frames = 10

        # Scaled waypoints params
        self.scaled_wpnts = None
        self.scaled_wpnts_msg = WpntArray()
        self.scaled_vmax = None
        self.scaled_max_idx = None
        self.scaled_max_s = None
        self.scaled_delta_s = None

        self.center_wpnts_msg = WpntArray()
        self.outer_lane_wpnts_msg = WpntArray()
        self.inner_lane_wpnts_msg = WpntArray()
        # Middle lane: the centerline itself as a selectable third lane.
        self.middle_lane_wpnts_msg = WpntArray()
        self.center_wpnts_received = False

        # Cached lane polylines for the fixed-rate lane-marker publisher (set in generate_lanes).
        self._center_xy = None
        self._outer_xy = None
        self._inner_xy = None

        # Updated waypoints params
        self.wpnts_updated = None
        self.max_s_updated = None

        # Obstalces params
        self.obs_perception = ObstacleArray()
        self.obs_prediction = ObstacleArray()

        # Solver params
        self.width_car = 0.30
        self.safety_margin = 0.1
        self.back_to_raceline_before = 3.0
        self.back_to_raceline_after = 3.0
        self.obs_traj_tresh = 2.0

        # Dynamic sovler params
        self.down_sampled_delta_s = 0.1

        # Evasion waypoint speed = raceline (scaled) speed at same s x this factor.
        self.evasion_speed_scale = 0.8

        # State variables (filled by callbacks)
        self.current_s = None
        self.current_d = None
        self.current_x = None
        self.current_vs = None  # ego frenet s-velocity

        # Dynamic reconf params (defaults from cfg/dyn_change_tuner.cfg)
        self.evasion_dist = 0.3
        self.spline_bound_mindist = 0.3
        # Max allowed |current_d - evasion_d| at the car before the evasion path is rejected.
        self.max_evasion_start_offset = 0.8
        self.lane_block_width = 0.3
        self.obs_pass_margin = 0.3

        self.min_lane_switch_gap = 1.0

        self.lane_switch_hold_m = 1.0

        # Transition slowdown: speed scale at the point of maximum lateral movement (1.0 = no slowdown).
        self.transition_speed_scale = 0.7

        # Constant-velocity projection of dynamic obstacles to where the opponent will be at intercept.
        self.dyn_proj_max_tau = 1.0   # [s] intercept-time cap for the projection
        self.dyn_proj_min_vs = 0.3    # [m/s] ignore vs below this (tracker noise floor)

        # Symmetric lane perturbation (centerline +/- lane_offset -> outer/inner lanes).
        # Adjustable live via rqt: changing it regenerates the two lanes.
        self.lane_offset = 0.3

        # Require fresh GP prediction before overtaking. Without it we only trail.
        # Prediction is considered stale if the latest prediction msg is older than this.
        self.pred_timeout = 0.5  # [s]
        self.last_pred_stamp = None
        # Default True (trail) until told otherwise, so a missing predictor never enables overtaking.
        self.force_trailing = True

        # Only clear the avoidance markers after this many consecutive failed/idle frames, so a
        # single dropped frame doesn't make the markers flicker.
        self.no_evasion_count = 0
        self.clear_after_frames = 5

        self.converter = None
        self.global_waypoints = None

        # CCMA init
        self.ccma = CCMA(w_ma=10, w_cc=5)

        # ROS Parameters
        self.declare_parameter('measure', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.measure = self.get_parameter('measure').get_parameter_value().bool_value

        # Show the matplotlib lane-visualization window in generate_lanes().
        # Default OFF: it blocks on plt.show() at startup and pops a GUI window.
        self.declare_parameter('vis', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.vis = self.get_parameter('vis').get_parameter_value().bool_value

        # Without an opponent-prediction publisher the GP-freshness overtake gate is bypassed by default.
        self.declare_parameter('require_prediction', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.require_prediction = self.get_parameter('require_prediction').get_parameter_value().bool_value

        # Also consider static obstacles for lane-change evasion.
        self.declare_parameter('avoid_static_obs', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.avoid_static_obs = self.get_parameter('avoid_static_obs').get_parameter_value().bool_value

        # Allow the centerline ("middle" lane) as a third overtake option.
        self.declare_parameter('use_middle_lane', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.use_middle_lane = self.get_parameter('use_middle_lane').get_parameter_value().bool_value

        # Anchor the evasion ease-in at the car instead of at the obstacle.
        self.declare_parameter('early_lane_commit', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.early_lane_commit = self.get_parameter('early_lane_commit').get_parameter_value().bool_value

        # Multi-group weave: obstacle groups separated by >= min_lane_switch_gap each get their own lane decision.
        self.declare_parameter('allow_lane_weave', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.allow_lane_weave = self.get_parameter('allow_lane_weave').get_parameter_value().bool_value

        # Constant-velocity projection of dynamic obstacles (false = plan on current positions).
        self.declare_parameter('dyn_cv_projection', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.dyn_cv_projection = self.get_parameter('dyn_cv_projection').get_parameter_value().bool_value

        # If false, the max_evasion_start_offset check only warns instead of rejecting the path.
        self.declare_parameter('enforce_start_offset', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.enforce_start_offset = self.get_parameter('enforce_start_offset').get_parameter_value().bool_value

        # Dynamic reconfigure -> declared ROS2 parameters
        param_dicts = [
            {
                'name': 'evasion_dist',
                'default': self.evasion_dist,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Orthogonal distance of the apex to the obstacle",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.25, step=0.001)],
                ),
            },
            {
                'name': 'safety_margin',
                'default': self.safety_margin,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Safety margin around the vehicle and obstacles",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.001)],
                ),
            },
            {
                'name': 'back_to_raceline_before',
                'default': self.back_to_raceline_before,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Distance before the obstacle used to return to the raceline",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.001)],
                ),
            },
            {
                'name': 'back_to_raceline_after',
                'default': self.back_to_raceline_after,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Distance after the obstacle used to return to the raceline",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.001)],
                ),
            },
            {
                'name': 'obs_traj_tresh',
                'default': self.obs_traj_tresh,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Threshold of the obstacle towards raceline to be considered for evasion",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.001)],
                ),
            },
            {
                'name': 'spline_bound_mindist',
                'default': self.spline_bound_mindist,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Splines may never be closer to the track bounds than this param in meters",
                    floating_point_range=[FloatingPointRange(from_value=0.05, to_value=1.0, step=0.001)],
                ),
            },
            {
                'name': 'lane_offset',
                'default': self.lane_offset,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Symmetric perturbation of the centerline to build outer/inner lanes [m]. Live editable, regenerates lanes.",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.001)],
                ),
            },
            {
                'name': 'pred_timeout',
                'default': self.pred_timeout,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Overtake only if a GP prediction arrived within this time, else trail [s]",
                    floating_point_range=[FloatingPointRange(from_value=0.05, to_value=5.0, step=0.001)],
                ),
            },
            {
                'name': 'max_evasion_start_offset',
                'default': self.max_evasion_start_offset,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Reject the evasion path if |current_d - evasion_d| at the car exceeds this [m]. Larger allows starting an evasion while closer to the opponent.",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.001)],
                ),
            },
            {
                'name': 'lane_block_width',
                'default': self.lane_block_width,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Lane counts as blocked if an obstacle d_center is within this of the eased path d at its s [m]. Keep > state machine lateral_width_ot_m (0.3).",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.001)],
                ),
            },
            {
                'name': 'evasion_speed_scale',
                'default': self.evasion_speed_scale,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Evasion waypoint speed = scaled raceline speed at same s x this factor",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.5, step=0.001)],
                ),
            },
            {
                'name': 'min_lane_switch_gap',
                'default': self.min_lane_switch_gap,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Weave: min free s-gap between obstacle groups to allow a lane switch between them [m]; smaller gaps merge into one group",
                    floating_point_range=[FloatingPointRange(from_value=0.5, to_value=15.0, step=0.001)],
                ),
            },
            {
                'name': 'transition_speed_scale',
                'default': self.transition_speed_scale,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Speed scale at max lateral movement (ease-in/blend/ease-out); 1.0 = no slowdown",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.0, step=0.001)],
                ),
            },
            {
                'name': 'lane_switch_hold_m',
                'default': self.lane_switch_hold_m,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Weave: hold full lane offset this far past each group's end before blending to the next lane [m] (capped at half the gap)",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=8.0, step=0.001)],
                ),
            },
            {
                'name': 'dyn_proj_max_tau',
                'default': self.dyn_proj_max_tau,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="CV projection: cap on the intercept-time used to shift dynamic obstacles forward [s]",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.001)],
                ),
            },
            {
                'name': 'dyn_proj_min_vs',
                'default': self.dyn_proj_min_vs,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="CV projection: ignore dynamic obstacles slower than this [m/s] (vs noise floor)",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3.0, step=0.001)],
                ),
            },
        ]
        self.declare_all_parameters(param_dicts=param_dicts)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        # Publishers
        self.mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers_sqp", QoSProfile(depth=10))
        self.evasion_pub = self.create_publisher(OTWpntArray, "/planner/avoidance/otwpnts", QoSProfile(depth=10))
        self.merger_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/merger", QoSProfile(depth=10))
        if self.measure:
            self.measure_pub = self.create_publisher(Float32, "/planner/pspliner_sqp/latency", QoSProfile(depth=10))

        self.spline_sample_pub = self.create_publisher(MarkerArray, "/spline_sample_points", QoSProfile(depth=10))

        # Lane visualization (always published, independent of the `vis` matplotlib flag).
        # Outer/inner perturbed lanes in distinct colors + how far each is offset from center.
        self.lane_mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/lane_markers", QoSProfile(depth=10))
        self.lane_offset_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/lane_offset", QoSProfile(depth=10))
        # Large spheres at the avoidance start/end so we can see exactly which points are picked.
        self.avoidance_pts_pub = self.create_publisher(MarkerArray, "/planner/avoidance/start_end_markers", QoSProfile(depth=10))

        # Subscribers
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.obs_perception_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/frenet/odom", self.state_frenet_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom", self.state_cartesian_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/local_waypoints", self.behavior_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/centerline_waypoints", self.center_wpnts_cb, QoSProfile(depth=10))

        # Make the startup waits visible: the node blocks silently here if the base system
        # (global_republisher) is down or publish_centerline is off.
        self.get_logger().info("[LC] waiting for /global_waypoints (FrenetConverter init)...")
        self.converter = self.initialize_converter()

        self.map_filter = GridFilter(node=self, map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(7)

        # Wait for the centerline waypoints, then generate the inner/outer lanes
        self.get_logger().info("[LC] waiting for /centerline_waypoints (needs publish_centerline:=true on global_republisher)...")
        self.wait_for_message_attr('center_wpnts_received')
        self.get_logger().info("[LC] centerline received, generating lanes")
        self.generate_lanes(center_wpnts=self.center_wpnts_msg)

        # Wait for critical messages and services
        self.get_logger().info("[OBS Spliner] Waiting for messages and services...")
        self.wait_for_loop_messages()
        self.get_logger().info("[OBS Spliner] Ready!")

        # Main loop timer at 20 Hz
        self.create_timer(1.0 / 20.0, self.loop)

        # Republish the lane markers + offset at a fixed rate so they stay visible in rviz
        # without touching rqt or regenerating the lanes.
        self.lane_viz_hz = 5.0
        self.create_timer(1.0 / self.lane_viz_hz, self.lane_viz_loop)

    #################### DYNAMIC PARAMS ####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def dyn_param_cb(self, params: List[Parameter]):
        regenerate_lanes = False
        for param in params:
            if param.name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param.name == 'safety_margin':
                self.safety_margin = param.value
            elif param.name == 'back_to_raceline_before':
                self.back_to_raceline_before = param.value
            elif param.name == 'back_to_raceline_after':
                self.back_to_raceline_after = param.value
            elif param.name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param.name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value
            elif param.name == 'lane_offset':
                if param.value != self.lane_offset:
                    regenerate_lanes = True
                self.lane_offset = param.value
            elif param.name == 'pred_timeout':
                self.pred_timeout = param.value
            elif param.name == 'max_evasion_start_offset':
                self.max_evasion_start_offset = param.value
            elif param.name == 'lane_block_width':
                self.lane_block_width = param.value
            elif param.name == 'avoid_static_obs':
                self.avoid_static_obs = param.value
            elif param.name == 'use_middle_lane':
                self.use_middle_lane = param.value
            elif param.name == 'early_lane_commit':
                self.early_lane_commit = param.value
            elif param.name == 'evasion_speed_scale':
                self.evasion_speed_scale = param.value
            elif param.name == 'allow_lane_weave':
                self.allow_lane_weave = param.value
            elif param.name == 'min_lane_switch_gap':
                self.min_lane_switch_gap = param.value
            elif param.name == 'transition_speed_scale':
                self.transition_speed_scale = param.value
            elif param.name == 'lane_switch_hold_m':
                self.lane_switch_hold_m = param.value
            elif param.name == 'dyn_cv_projection':
                self.dyn_cv_projection = param.value
            elif param.name == 'enforce_start_offset':
                self.enforce_start_offset = param.value
            elif param.name == 'dyn_proj_max_tau':
                self.dyn_proj_max_tau = param.value
            elif param.name == 'dyn_proj_min_vs':
                self.dyn_proj_min_vs = param.value

        # Rebuild the outer/inner lanes with the new offset. Only once the centerline
        # is available (i.e. after the initial generate_lanes at startup).
        if regenerate_lanes and self.center_wpnts_received:
            self.generate_lanes(center_wpnts=self.center_wpnts_msg)
            self.get_logger().info(f"[Planner] Regenerated lanes with lane_offset={self.lane_offset} [m]")

        self.get_logger().info(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Evasion apex distance: {self.evasion_dist} [m],\n"
            f" Safety margin: {self.safety_margin} [m],\n"
            f" Back to raceline before: {self.back_to_raceline_before} [m],\n"
            f" Back to raceline after: {self.back_to_raceline_after} [m],\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
            f" Lane offset: {self.lane_offset} [m]\n"
            f" Prediction timeout: {self.pred_timeout} [s]\n"
        )
        return SetParametersResult(successful=True)

    ### Callbacks ###
    def obs_perception_cb(self, data: ObstacleArray):
        self.obs_perception = data
        if self.avoid_static_obs:
            self.obs_perception.obstacles = list(data.obstacles)
        else:
            self.obs_perception.obstacles = [obs for obs in data.obstacles if obs.is_static == False]

    def obs_prediction_cb(self, data: ObstacleArray):
        self.obs_prediction = data

    def force_trailing_cb(self, data: Bool):
        self.force_trailing = data.data

    def prediction_is_fresh(self) -> bool:
        """True if a non-empty GP prediction arrived within the last pred_timeout seconds."""
        if self.last_pred_stamp is None:
            return False
        dt = (self.get_clock().now() - self.last_pred_stamp).nanoseconds * 1e-9
        return dt <= self.pred_timeout

    def state_frenet_cb(self, data: Odometry):
        self.current_s = data.pose.pose.position.x
        self.current_d = data.pose.pose.position.y
        self.current_vs = data.twist.twist.linear.x  # ego frenet s-velocity

    def state_cartesian_cb(self, data: Odometry):
        self.current_x = data.pose.pose.position.x

    def gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        # FrenetConverter requires psi at construction.
        self.global_waypoints_psi = np.array([wpnt.psi_rad for wpnt in data.wpnts])

    def scaled_wpnts_cb(self, data: WpntArray):
        self.scaled_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])
        self.scaled_wpnts_msg = data
        v_max = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
        if self.scaled_vmax != v_max:
            self.scaled_vmax = v_max
            self.scaled_max_idx = data.wpnts[-1].id
            self.scaled_max_s = data.wpnts[-1].s_m
            self.scaled_delta_s = data.wpnts[1].s_m - data.wpnts[0].s_m

    def updated_wpnts_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        self.max_s_updated = self.wpnts_updated[-1].s_m

    def center_wpnts_cb(self, data: WpntArray):
        self.center_wpnts_msg = data
        self.center_wpnts_received = True

    def behavior_cb(self, data: WpntArray):
        # Local waypoints come from the state machine's /local_waypoints (WpntArray).
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])

    ### Common Functions ###
    def wait_for_message_attr(self, attr_name: str):
        """Spin until the named instance attribute has been set by a callback."""
        while not getattr(self, attr_name, False):
            rclpy.spin_once(self)

    def wait_for_loop_messages(self):
        """Spin until all critical messages have been received before starting the loop."""
        while (self.scaled_wpnts is None or self.current_x is None
               or self.local_wpnts is None):
            rclpy.spin_once(self)

    def initialize_converter(self) -> "FrenetConverter":
        """
        Initialize the FrenetConverter object"""
        # Wait for the global waypoints to arrive
        while self.global_waypoints is None:
            rclpy.spin_once(self)

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.global_waypoints[:, 0], self.global_waypoints[:, 1],
                                    self.global_waypoints_psi)
        self.get_logger().info("[Spliner] initialized FrenetConverter object")

        return converter

    def obstacle_preprocessing(self, obs: ObstacleArray):
        obs.obstacles = sorted(obs.obstacles, key=lambda obs: obs.s_start)

        considered_obs = []
        for obs in obs.obstacles:
            # Keep the obstacle considered until the car has passed its rear edge (+obs_pass_margin).

            # Constant-velocity projection of dynamic obstacles: shift the box to where the opponent will be at intercept.
            if (self.dyn_cv_projection and self.current_vs is not None
                    and not obs.is_static and obs.vs > self.dyn_proj_min_vs):
                gap = (obs.s_center - self.current_s) % self.scaled_max_s
                if gap < self.scaled_max_s / 2:  # never project obstacles BEHIND the car
                    rel_v = max(self.current_vs - obs.vs, 0.5)
                    tau = np.clip(gap / rel_v, 0.0, self.dyn_proj_max_tau)
                    shift = tau * obs.vs
                    obs.s_start = (obs.s_start + shift) % self.scaled_max_s
                    obs.s_center = (obs.s_center + shift) % self.scaled_max_s
                    obs.s_end = (obs.s_end + shift) % self.scaled_max_s
                    proj_len = (obs.s_end - obs.s_start) % self.scaled_max_s
                    t_pass = (proj_len + self.back_to_raceline_after) / rel_v  # opponent keeps advancing while we pass
                    obs.s_end = (obs.s_end + obs.vs * t_pass) % self.scaled_max_s

            obs_len = (obs.s_end - obs.s_start) % self.scaled_max_s
            dist = (obs.s_end + self.obs_pass_margin - self.current_s) % self.scaled_max_s
            if dist < self.lookahead + obs_len + self.obs_pass_margin and abs(obs.d_center - self.current_d) < self.obs_traj_tresh:
                considered_obs.append(obs)

        return considered_obs

    def _apply_side_hysteresis(self, raw_side: str) -> str:
        if self.committed_side is None:
            self.committed_side = raw_side
            self.pending_side = None
            self.pending_count = 0
        elif raw_side == self.committed_side:
            self.pending_side = None
            self.pending_count = 0
        else:
            if raw_side == self.pending_side:
                self.pending_count += 1
            else:
                self.pending_side = raw_side
                self.pending_count = 1
            if self.pending_count >= self.side_switch_frames:
                self.committed_side = raw_side
                self.pending_side = None
                self.pending_count = 0
        return self.committed_side

    def more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        left_gap = gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left
        right_gap = gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right
        min_space = self.spline_bound_mindist + self.width_car / 2 + self.safety_margin

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - (self.width_car / 2 + self.safety_margin + 0.2)
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0 and right_gap < abs(d_apex_right):
                d_apex_right = 0
            return "right", d_apex_right, left_gap, right_gap

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + (self.width_car / 2 + self.safety_margin + 0.2)
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0 and left_gap < abs(d_apex_left):
                d_apex_left = 0
            return "left", d_apex_left, left_gap, right_gap
        elif left_gap < min_space and right_gap < min_space:
            # Both sides too narrow: genuinely no room -> don't overtake.
            return None, 0.0, left_gap, right_gap
        else:
            # Both sides wide enough: pick the wider one.
            if left_gap >= right_gap:
                d_apex_left = obstacle.d_left + (self.width_car / 2 + self.safety_margin + 0.2)
                if d_apex_left < 0 and left_gap < abs(d_apex_left):
                    d_apex_left = 0
                return "left", d_apex_left, left_gap, right_gap
            else:
                d_apex_right = obstacle.d_right - (self.width_car / 2 + self.safety_margin + 0.2)
                if d_apex_right > 0 and right_gap < abs(d_apex_right):
                    d_apex_right = 0
                return "right", d_apex_right, left_gap, right_gap

    ### Visualize SPL Rviz ###
    def visualize_dynamic_spliner(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        if self.mrks_pub.get_subscription_count() == 0:  # rviz-off: skip marker
            return
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            # DELETEALL first so a shorter path this frame leaves no leftover cylinders.
            del_mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            for i in range(len(evasion_s)):
                mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
                mrk.type = mrk.CYLINDER
                mrk.scale.x = 0.1
                mrk.scale.y = 0.1
                # Keep a minimum height so the evasion path always shows in rviz.
                mrk.scale.z = max(evasion_v[i] / self.scaled_vmax, 0.15)
                mrk.color.a = 1.0
                mrk.color.g = 0.13
                mrk.color.r = 0.63
                mrk.color.b = 0.94

                mrk.id = i
                mrk.pose.position.x = evasion_x[i]
                mrk.pose.position.y = evasion_y[i]
                mrk.pose.position.z = mrk.scale.z / 2
                mrk.pose.orientation.w = 1.0
                mrk.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
                mrks.markers.append(mrk)
            self.mrks_pub.publish(mrks)

    def visualize_spline_samples(self, x_vals, y_vals):
        if self.spline_sample_pub.get_subscription_count() == 0:  # rviz-off: skip marker
            return
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(x_vals, y_vals)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "spline_samples"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1  # optional: small height
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # stays for 0.2 sec
            marker_array.markers.append(marker)

        self.spline_sample_pub.publish(marker_array)

    ### Lane Change Avoidance ###
    def _unwrap_forward(self, s: float, ref_s: float) -> float:
        """Return s expressed as ref_s + forward-distance, so it is always >= ref_s and monotonic
        across the s=0 seam. A point up to `back_tol` behind ref_s (perception jitter) is clamped
        to ref_s instead of being wrapped a whole lap forward (which flashed the path backwards)."""
        back_tol = 1.0
        fwd = (s - ref_s) % self.scaled_max_s
        if fwd > self.scaled_max_s - back_tol:
            fwd = 0.0
        return ref_s + fwd

    def lane_change(self, considered_obs: list, cur_s: float):
        # Unwrap every obstacle's s forward of the car so the index math and s_avoidance stay monotonic.
        for obs in considered_obs:
            obs.s_start = self._unwrap_forward(obs.s_start, self.current_s)
            obs.s_end = self._unwrap_forward(obs.s_end, self.current_s)
            obs.s_center = self._unwrap_forward(obs.s_center, self.current_s)
            if obs.s_end < obs.s_start:
                obs.s_end += self.scaled_max_s

        # ---- Multi-group weave: obstacle groups separated by free s each get their own lane decision.
        groups = self._split_obstacle_groups(considered_obs)
        single_group = len(groups) == 1

        # Entry-ease anchors for the FIRST group (same early_lane_commit logic as before;
        # also bounds the occupancy entry window of the first group in weave mode).
        first_start = min(o.s_start for o in groups[0])
        ramp_in = self.back_to_raceline_before
        ramp_out = self.back_to_raceline_after
        if self.early_lane_commit:
            ease_in_start = self.current_s
            ease_in_end = min(self.current_s + ramp_in, first_start)
        else:
            ease_in_start = first_start - ramp_in
            ease_in_end = first_start
        ease_in_len = max(ease_in_end - ease_in_start, 1e-6)

        # Per-group lane decision; prefer=previous group's lane biases the tie-break toward fewer transitions.
        plan = []  # [{'obs', 'start', 'end', 'lane', 'candidates', 'outside', 'max_kappa'}]
        prev_lane = self.committed_side
        for gi, g_obs in enumerate(groups):
            next_start = min(o.s_start for o in groups[gi + 1]) if gi + 1 < len(groups) else None
            prev_end = plan[-1]['end'] if plan else None
            lane, g_start, g_end, info = self._choose_lane_for_group(
                g_obs, gi, first_group=(gi == 0), single_group=single_group,
                prefer=prev_lane, next_start=next_start, prev_end=prev_end,
                ease_in_start=ease_in_start)
            if lane is None:
                self.get_logger().warn(f"[LC reject] group {gi}: {info['reason']}",
                                       throttle_duration_sec=1.0)
                return [], [], [], [], []
            plan.append({'obs': g_obs, 'start': g_start, 'end': g_end, 'lane': lane,
                         'candidates': info['candidates'], 'outside': info['outside'],
                         'max_kappa': info['max_kappa']})
            prev_lane = lane

        # Hysteresis applies to the FIRST group's lane only -- that is what steers the car
        # right now; later groups become "first" on subsequent replans and get it then.
        raw_side = plan[0]['lane']
        preferred_side = self._apply_side_hysteresis(raw_side)
        if preferred_side not in plan[0]['candidates']:
            # Committed side's lane is blocked; raw_side is viable by construction, so force-commit it immediately.
            self.get_logger().warn(
                f"[LC] committed side '{preferred_side}' lane blocked -> forcing '{raw_side}'")
            self.committed_side = raw_side
            self.pending_side = None
            self.pending_count = 0
            preferred_side = raw_side
        plan[0]['lane'] = preferred_side

        # Elongation with the final lane per group; in weave mode capped so it never eats into the next gap.
        for gi, g in enumerate(plan):
            if g['lane'] == g['outside']:
                next_start = plan[gi + 1]['start'] if gi + 1 < len(plan) else None
                for o in g['obs']:
                    obs_len = o.s_end - o.s_start
                    enlongation = obs_len * g['max_kappa'] * (self.width_car + self.evasion_dist)
                    enlongation = min(enlongation, self.scaled_max_s * 0.15)
                    new_end = o.s_end + enlongation
                    if next_start is not None:
                        new_end = min(new_end, next_start - self.min_lane_switch_gap)
                    o.s_end = max(o.s_end, new_end)
                g['end'] = max(o.s_end for o in g['obs'])

        if len(plan) > 1:
            seq = " -> ".join(f"{g['lane']}@[{g['start']:.1f},{g['end']:.1f}]" for g in plan)
            self.get_logger().info(f"[LC] weave plan: {seq}", throttle_duration_sec=1.0)

        # ---- Monotonic-s sampling, piecewise over the group chain: ease in, hold, blend between lanes, ease out.
        lane_wpnts_map = {"left": self.outer_lane_wpnts_msg.wpnts,
                          "middle": self.middle_lane_wpnts_msg.wpnts,
                          "right": self.inner_lane_wpnts_msg.wpnts}

        start_s = self.current_s
        obs_start_u = plan[0]['start']
        obs_end_u = plan[-1]['end']
        end_s = obs_end_u + self.back_to_raceline_after
        end_s = min(end_s, start_s + self.scaled_max_s * 0.5)

        n_samples = max(int((end_s - start_s) / self.scaled_delta_s), 5)
        s_lin = np.linspace(start_s, end_s, n_samples)  # monotonic, unwrapped

        # Each group's lane d along the full s_lin; the blends below index into these.
        lane_d_profiles = [self._lane_d_at_s(lane_wpnts_map[g['lane']], s_lin) for g in plan]

        # Transition speed scale: slow down wherever the path moves laterally; 1.0 at both ends of a transition.
        d_arr = np.zeros_like(s_lin)
        speed_scale_arr = np.ones_like(s_lin)
        ts = self.transition_speed_scale
        for i, s in enumerate(s_lin):
            if s <= ease_in_start or s > obs_end_u + ramp_out:
                d_arr[i] = 0.0
            elif s < ease_in_end:
                w = 0.5 * (1 - np.cos(np.pi * (s - ease_in_start) / ease_in_len))
                d_arr[i] = w * lane_d_profiles[0][i]
                speed_scale_arr[i] = 1.0 - (1.0 - ts) * np.sin(np.pi * w)
            elif s > obs_end_u:
                w = 0.5 * (1 + np.cos(np.pi * (s - obs_end_u) / ramp_out))
                d_arr[i] = w * lane_d_profiles[-1][i]
                speed_scale_arr[i] = 1.0 - (1.0 - ts) * np.sin(np.pi * w)
            else:
                # Find the active group; blend across the gap if s sits between two groups.
                gi = 0
                while gi + 1 < len(plan) and s > plan[gi]['end'] and s >= plan[gi + 1]['start']:
                    gi += 1
                if gi + 1 < len(plan) and s > plan[gi]['end']:
                    # Hold the full lane-i offset for lane_switch_hold_m past the group end, then blend to lane i+1.
                    gap = max(plan[gi + 1]['start'] - plan[gi]['end'], 1e-6)
                    hold_eff = min(self.lane_switch_hold_m, 0.5 * gap)
                    blend_start = plan[gi]['end'] + hold_eff
                    if s <= blend_start:
                        d_arr[i] = lane_d_profiles[gi][i]
                    else:
                        blend_len = max(plan[gi + 1]['start'] - blend_start, 1e-6)
                        w = 0.5 * (1 - np.cos(np.pi * (s - blend_start) / blend_len))
                        d_arr[i] = (1 - w) * lane_d_profiles[gi][i] + w * lane_d_profiles[gi + 1][i]
                        speed_scale_arr[i] = 1.0 - (1.0 - ts) * np.sin(np.pi * w)
                else:
                    d_arr[i] = lane_d_profiles[gi][i]

        s_wrapped = s_lin % self.scaled_max_s
        resp = self.converter.get_cartesian(s_wrapped, d_arr)
        resp = resp.T if resp.ndim == 2 else resp
        samples = np.asarray(resp, dtype=float).reshape(-1, 2)
        if samples.shape[0] < 3 or not np.all(np.isfinite(samples)):
            return [], [], [], [], []

        for i in range(samples.shape[0]):
            inside = self.map_filter.is_point_inside(samples[i, 0], samples[i, 1])

            if not inside:
                self.get_logger().warn(
                    f"[LC reject] sample ({samples[i, 0]:.2f}, {samples[i, 1]:.2f}) outside eroded map "
                    f"-> whole path dropped (lane_offset={self.lane_offset:.2f} may not fit this corridor)",
                    throttle_duration_sec=1.0)
                evasion_x = []
                evasion_y = []
                evasion_s = []
                evasion_d = []
                evasion_v = []
                return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

        self.visualize_spline_samples(samples[:, 0], samples[:, 1])

        smoothed_xy_points = self.ccma.filter(samples)
        smoothed_sd_points = self.converter.get_frenet(smoothed_xy_points[:, 0], smoothed_xy_points[:, 1])
        evasion_x = np.asarray(smoothed_xy_points[:, 0])
        evasion_y = np.asarray(smoothed_xy_points[:, 1])
        evasion_d = np.asarray(smoothed_sd_points[1])
        # get_frenet returns wrapped s; CCMA can locally reorder s near the apex. Unwrap, then
        # keep only strictly-increasing s to drop duplicate/reversing points (NaN source).
        evasion_s_raw = np.asarray(smoothed_sd_points[0])
        evasion_s = start_s + (evasion_s_raw - start_s) % self.scaled_max_s

        keep = np.ones(len(evasion_s), dtype=bool)
        last_s = -np.inf
        for i in range(len(evasion_s)):
            if evasion_s[i] - last_s > 1e-4:
                last_s = evasion_s[i]
            else:
                keep[i] = False
        evasion_x = evasion_x[keep]
        evasion_y = evasion_y[keep]
        evasion_s = evasion_s[keep]
        evasion_d = evasion_d[keep]
        evasion_coords = np.column_stack((evasion_x, evasion_y))

        # Guard: drop coincident xy points (zero-length segments -> NaN psi/kappa).
        if len(evasion_coords) >= 2:
            seg = np.linalg.norm(np.diff(evasion_coords, axis=0), axis=1)
            keep = np.concatenate([[True], seg > 1e-6])
            if not np.all(keep):
                evasion_x = evasion_x[keep]
                evasion_y = evasion_y[keep]
                evasion_s = evasion_s[keep]
                evasion_d = evasion_d[keep]
                evasion_coords = evasion_coords[keep]
        if len(evasion_coords) < 3 or not np.all(np.isfinite(evasion_coords)):
            return [], [], [], [], []

        # Unwrapped copy kept for the transition speed-scale interp below (aligns with s_lin).
        evasion_s_unwrapped = evasion_s.copy()
        evasion_s = evasion_s % self.scaled_max_s

        evasion_psi, evasion_kappa = tph.calc_head_curv_num.calc_head_curv_num(
            path=evasion_coords,
            el_lengths=0.1 * np.ones(len(evasion_coords) - 1),
            is_closed=False
        )
        evasion_psi += np.pi / 2
        # Use the scaled raceline speed at each s, scaled by evasion_speed_scale.
        scaled_v_arr = np.array([w.vx_mps for w in self.scaled_wpnts_msg.wpnts])
        evasion_v = self.evasion_speed_scale * np.interp(
            evasion_s, self.scaled_wpnts[:, 0], scaled_v_arr)
        # Transition slowdown: scale the speed down where the path is moving laterally.
        evasion_v = evasion_v * np.interp(evasion_s_unwrapped, s_lin, speed_scale_arr)

        # Steep-start rejection is opt-in (enforce_start_offset); otherwise exceeding the offset only warns.
        temp_idx = np.argmin([abs(evs - self.current_s) for evs in evasion_s])
        start_offset = abs(self.current_d - evasion_d[temp_idx])
        if start_offset > self.max_evasion_start_offset:
            self.get_logger().warn(
                f"[LC{' reject' if self.enforce_start_offset else ''}] start offset large: "
                f"|current_d - path_d| = {start_offset:.2f} > max_evasion_start_offset "
                f"{self.max_evasion_start_offset:.2f}"
                + ("" if self.enforce_start_offset else " (enforce_start_offset=false -> publishing anyway)"),
                throttle_duration_sec=1.0)
            if self.enforce_start_offset:
                evasion_x = []
                evasion_y = []
                evasion_s = []
                evasion_d = []
                evasion_v = []
                return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

        # Create a new evasion waypoint message
        evasion_wpnts_msg = OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
        # Tag the chosen lane sequence, e.g. "right->left".
        evasion_wpnts_msg.ot_side = "->".join(g['lane'] for g in plan)
        evasion_wpnts_msg.ot_line = "lane_change"
        evasion_wpnts = []
        evasion_wpnts = [Wpnt(id=len(evasion_wpnts), s_m=s, d_m=d, x_m=x, y_m=y, psi_rad=p, kappa_radpm=k, vx_mps=v) for x, y, s, d, p, k, v in zip(evasion_x, evasion_y, evasion_s, evasion_d, evasion_psi, evasion_kappa, evasion_v)]
        evasion_wpnts_msg.wpnts = evasion_wpnts

        self.evasion_pub.publish(evasion_wpnts_msg)
        self.visualize_dynamic_spliner(evasion_s, evasion_d, evasion_x, evasion_y, evasion_v)
        # Only publish the start/end debug spheres once the path actually succeeds, so they track
        # the published path instead of flickering on every early-return frame.
        self.publish_start_end_markers(start_s, obs_start_u, obs_end_u, end_s)

        return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

    def resample_lane(self, xy: np.ndarray, resolution: float = 0.1) -> np.ndarray:
        deltas = np.diff(xy, axis=0)
        dists = np.hypot(deltas[:, 0], deltas[:, 1])
        s = np.concatenate([[0], np.cumsum(dists)])
        s_new = np.arange(0, s[-1], resolution)
        x_new = np.interp(s_new, s, xy[:, 0])
        y_new = np.interp(s_new, s, xy[:, 1])
        return np.stack([x_new, y_new], axis=1)

    def _fill_lane_msg(self, msg: WpntArray, lane_xy_resampled: np.ndarray, lane_s, lane_d):
        """Compute psi/kappa for one lane polyline and (re)fill its WpntArray in place.

        Clears msg.wpnts first so a live regeneration (lane_offset change) doesn't append
        onto the old lane.
        """
        lane_psi, lane_kappa = tph.calc_head_curv_num.calc_head_curv_num(
                path=lane_xy_resampled,
                el_lengths=0.1 * np.ones(len(lane_xy_resampled) - 1),
                is_closed=False,
                stepsize_curv_preview=5.0,
                stepsize_curv_review=5.0
            )
        lane_psi += np.pi / 2

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.wpnts = []
        for i in range(len(lane_xy_resampled)):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = lane_xy_resampled[i, 0]
            wpnt.y_m = lane_xy_resampled[i, 1]
            wpnt.s_m = lane_s[i]
            wpnt.d_m = lane_d[i]
            wpnt.psi_rad = lane_psi[i]
            wpnt.kappa_radpm = lane_kappa[i]
            wpnt.d_left = 0.0
            wpnt.d_right = 0.0
            wpnt.vx_mps = 0.0
            wpnt.ax_mps2 = 0.0
            msg.wpnts.append(wpnt)

    def _lane_d_at_s(self, lane_wpnts, s_query):
        # Interpolate a lane's d over raceline s. Lane s wraps [0, scaled_max_s); duplicate one
        # lap ahead so an UNWRAPPED s_query (can exceed scaled_max_s) still interpolates cleanly.
        ls = np.array([w.s_m for w in lane_wpnts])
        ld = np.array([w.d_m for w in lane_wpnts])
        order = np.argsort(ls)
        ls, ld = ls[order], ld[order]
        ls_ext = np.concatenate([ls, ls + self.scaled_max_s])
        ld_ext = np.concatenate([ld, ld])
        return np.interp(s_query, ls_ext, ld_ext)

    def _lane_blocked(self, lane_wpnts, span_start, span_end):
        """True if any tracked obstacle sits where the eased path onto this lane would pass.

        Iterates self.obs_perception (NOT considered_obs) so obstacles filtered out by
        obs_traj_tresh / lookahead but sitting in the lane still block it. Mirrors both the
        cosine-ease weight of the path build in lane_change() and the state machine's
        _check_ofree test |path_d - obs.d_center| < lateral_width_ot_m, so a lane that
        passes here yields a path the state machine will not veto. Wrap-safe via
        _unwrap_forward. Returns (blocked, blocking_obs | None).

        Reads raw perception positions on purpose: the CV projection in
        obstacle_preprocessing only shifts the deepcopied considered_obs, so lane
        occupancy here stays on current positions (conservative).
        """
        ramp_in = self.back_to_raceline_before
        ramp_out = self.back_to_raceline_after
        # Mirror of the ease anchors in lane_change(): with early_lane_commit the path is at full lane d up to span_end.
        if self.early_lane_commit:
            ease_in_start = self.current_s
            ease_in_end = min(self.current_s + ramp_in, span_start)
        else:
            ease_in_start = span_start - ramp_in
            ease_in_end = span_start
        ease_in_len = max(ease_in_end - ease_in_start, 1e-6)
        for obs in self.obs_perception.obstacles:
            obs_s = self._unwrap_forward(obs.s_center, self.current_s)
            if obs_s <= ease_in_start or obs_s > span_end + ramp_out:
                continue
            if obs_s < ease_in_end:     # entry ramp: path still easing from raceline to lane
                w = 0.5 * (1 - np.cos(np.pi * (obs_s - ease_in_start) / ease_in_len))
            elif obs_s > span_end:      # exit ramp: path easing back to the raceline
                w = 0.5 * (1 + np.cos(np.pi * (obs_s - span_end) / ramp_out)) if ramp_out > 0 else 1.0
            else:
                w = 1.0
            lane_d = float(self._lane_d_at_s(lane_wpnts, obs_s))
            if abs(w * lane_d - obs.d_center) < self.lane_block_width:
                return True, obs
        return False, None

    def _split_obstacle_groups(self, considered_obs: list) -> list:
        """Split (already unwrapped-forward) obstacles into groups separated by at least
        min_lane_switch_gap of free s -- enough room to transition between lanes, so each
        group can get its own lane decision (weave). With allow_lane_weave off everything
        stays in one group, reproducing the original single-decision behavior exactly."""
        obs_sorted = sorted(considered_obs, key=lambda o: o.s_start)
        groups = [[obs_sorted[0]]]
        for o in obs_sorted[1:]:
            cur_end = max(x.s_end for x in groups[-1])
            if self.allow_lane_weave and (o.s_start - cur_end) >= self.min_lane_switch_gap:
                groups.append([o])
            else:
                groups[-1].append(o)
        return groups

    def _lane_blocked_group(self, lane_wpnts, check_start, check_end):
        """Weave-mode occupancy test for ONE group's window: any perceived obstacle whose
        (unwrapped) s_center lies in [check_start, check_end] and whose d_center is within
        lane_block_width of the lane's d blocks the lane. Tested at full lane offset (w=1),
        which is conservative inside the half-transition margins at the window edges.
        Single-group plans keep using _lane_blocked (exact original ease-weighted test).
        The group-LOCAL window is what lets group 1 ignore an obstacle that only blocks
        group 2's stretch (the whole point of the weave)."""
        for obs in self.obs_perception.obstacles:
            obs_s = self._unwrap_forward(obs.s_center, self.current_s)
            if obs_s < check_start or obs_s > check_end:
                continue
            lane_d = float(self._lane_d_at_s(lane_wpnts, obs_s))
            if abs(lane_d - obs.d_center) < self.lane_block_width:
                return True, obs
        return False, None

    def _choose_lane_for_group(self, group_obs, gi, first_group, single_group, prefer,
                               next_start, prev_end, ease_in_start):
        """Lane decision for one obstacle group, run per group by the multi-group weave.
        Returns (lane|None, span_start, span_end, info); info carries
        candidates/outside/max_kappa ('reason' on reject). span_end is un-elongated;
        lane_change() elongates after hysteresis fixes the final lane."""
        span_start = min(o.s_start for o in group_obs)
        span_end = max(o.s_end for o in group_obs)

        # scaled_wpnts is [[s, d], ...]; match on the s column only. np.abs(2D-scalar).argmin()
        # would return a flattened index (~2x) and pick the wrong waypoint.
        scaled_s = self.scaled_wpnts[:, 0]
        idx_start = np.abs(scaled_s - span_start % self.scaled_max_s).argmin()
        idx_end = np.abs(scaled_s - span_end % self.scaled_max_s).argmin()
        gb_idxs = np.array(range(idx_start, idx_start + (idx_end - idx_start) % self.scaled_max_idx)) % self.scaled_max_idx
        if len(gb_idxs) < 20:
            gb_idxs = [int(group_obs[0].s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)]

        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        left_count = right_count = 0
        left_gap_sum = right_gap_sum = 0.0
        for obs in group_obs:
            side, _apex, left_gap, right_gap = self.more_space(obs, self.scaled_wpnts_msg.wpnts, gb_idxs)
            left_gap_sum += left_gap
            right_gap_sum += right_gap
            if side == "left":
                left_count += 1
            elif side == "right":
                right_count += 1

        # Gap averages over ALL scored obstacles for the occupancy-override gate.
        n_scored = max(len(group_obs), 1)
        left_gap_all = left_gap_sum / n_scored
        right_gap_all = right_gap_sum / n_scored
        left_gap_avg = left_gap_sum / left_count if left_count > 0 else 0
        right_gap_avg = right_gap_sum / right_count if right_count > 0 else 0

        # Vote winner under the original strict-majority rule, or None.
        vote_side = None
        if left_count > right_count and left_gap_avg > right_gap_avg and left_gap_avg > self.width_car:
            vote_side = "left"
        elif right_count > left_count and right_gap_avg > left_gap_avg and right_gap_avg > self.width_car:
            vote_side = "right"

        # Hypothetical elongated span end for the occupancy windows, capped so it never eats into the next gap.
        elong_cap = self.scaled_max_s * 0.15
        elong_end = max(
            o.s_end + min((o.s_end - o.s_start) * max_kappa * (self.width_car + self.evasion_dist), elong_cap)
            for o in group_obs)
        if next_start is not None:
            elong_end = max(min(elong_end, next_start - self.min_lane_switch_gap), span_end)
        span_end_left = elong_end if outside == "left" else span_end
        span_end_right = elong_end if outside == "right" else span_end
        # Middle is never the "outside" lane -> no elongated span.
        span_end_middle = span_end

        if single_group:
            # Exact original ease-weighted test over the whole (only) span.
            blocked_left, _ = self._lane_blocked(self.outer_lane_wpnts_msg.wpnts, span_start, span_end_left)
            blocked_right, _ = self._lane_blocked(self.inner_lane_wpnts_msg.wpnts, span_start, span_end_right)
            blocked_middle, _ = self._lane_blocked(self.middle_lane_wpnts_msg.wpnts, span_start, span_end_middle)
        else:
            # Group-local windows: from the entry anchor or preceding-gap midpoint to the following-gap midpoint.
            entry_chk = ease_in_start if first_group else (prev_end + span_start) / 2.0

            def _chk(lane_wpnts, s_end_side):
                if next_start is not None:
                    end_chk = (s_end_side + next_start) / 2.0
                else:
                    end_chk = s_end_side + self.back_to_raceline_after / 2.0
                blocked, _obs = self._lane_blocked_group(lane_wpnts, entry_chk, end_chk)
                return blocked

            blocked_left = _chk(self.outer_lane_wpnts_msg.wpnts, span_end_left)
            blocked_right = _chk(self.inner_lane_wpnts_msg.wpnts, span_end_right)
            blocked_middle = _chk(self.middle_lane_wpnts_msg.wpnts, span_end_middle)

        candidates = []
        if not blocked_left and left_gap_all > self.width_car:
            candidates.append("left")
        if not blocked_right and right_gap_all > self.width_car:
            candidates.append("right")
        # No gap_all analog for middle: _lane_blocked IS the middle clearance test.
        if self.use_middle_lane and not blocked_middle:
            candidates.append("middle")

        info = {'candidates': candidates, 'outside': outside, 'max_kappa': max_kappa}

        if not candidates:
            info['reason'] = (
                f"all lanes blocked/too narrow: "
                f"L blocked={blocked_left} gap={left_gap_all:.2f} | "
                f"M blocked={blocked_middle} (use_middle_lane={self.use_middle_lane}) | "
                f"R blocked={blocked_right} gap={right_gap_all:.2f} "
                f"(lane_block_width={self.lane_block_width:.2f})")
            return None, span_start, span_end, info

        # Same resolution as the original block; middle stays a LAST-RESORT fallback.
        side_candidates = [c for c in candidates if c != "middle"]
        if vote_side in side_candidates:
            lane = vote_side
        elif len(side_candidates) == 1:
            lane = side_candidates[0]
            self.get_logger().info(
                f"[LC] group {gi} occupancy override -> {lane} "
                f"(vote={vote_side}, L blocked={blocked_left}, R blocked={blocked_right})",
                throttle_duration_sec=1.0)
        elif len(side_candidates) == 2:
            # Both side lanes free but the vote was inconclusive: prefer the previous group's
            # lane (first group: the committed side) for stability, else the wider lane.
            lane = prefer if prefer in side_candidates \
                else ("left" if left_gap_all >= right_gap_all else "right")
        else:
            # Both side lanes blocked; candidates is non-empty here, so middle is free.
            lane = "middle"
            self.get_logger().info(
                f"[LC] group {gi} middle-lane fallback: both side lanes blocked "
                f"(L gap={left_gap_all:.2f}, R gap={right_gap_all:.2f})",
                throttle_duration_sec=1.0)

        return lane, span_start, span_end, info

    def generate_lanes(self, center_wpnts: WpntArray):
        original_center_wpnts = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in center_wpnts.wpnts])
        original_center_psi = np.array([wpnt.psi_rad for wpnt in center_wpnts.wpnts])

        min_center_left_gap = np.min([wpnt.d_left for wpnt in center_wpnts.wpnts])
        min_center_right_gap = np.min([wpnt.d_right for wpnt in center_wpnts.wpnts])

        normals = np.stack([np.sin(original_center_psi), -np.cos(original_center_psi)], axis=1)

        # Constant orthogonal perturbation of the centerline (live-tunable via lane_offset)
        outer_lane = original_center_wpnts + normals * self.lane_offset
        inner_lane = original_center_wpnts - normals * self.lane_offset

        outer_lane_resampled = self.resample_lane(outer_lane, resolution=0.1)
        inner_lane_resampled = self.resample_lane(inner_lane, resolution=0.1)

        outer_s, outer_d = self.converter.get_frenet(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1])
        inner_s, inner_d = self.converter.get_frenet(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1])

        # Middle lane = the centerline itself, run through the same resample+frenet pipeline as the side lanes.
        middle_lane_resampled = self.resample_lane(original_center_wpnts, resolution=0.1)
        middle_s, middle_d = self.converter.get_frenet(middle_lane_resampled[:, 0], middle_lane_resampled[:, 1])

        # Downstream treats outer_lane as the left (+d) lane; swap if the normal convention put it on the wrong side.
        if np.mean(outer_d) < np.mean(inner_d):
            outer_lane_resampled, inner_lane_resampled = inner_lane_resampled, outer_lane_resampled
            outer_s, inner_s = inner_s, outer_s
            outer_d, inner_d = inner_d, outer_d
            self.get_logger().info("[Planner] outer/inner lanes swapped to match frenet +d=left convention")

        self._fill_lane_msg(self.outer_lane_wpnts_msg, outer_lane_resampled, outer_s, outer_d)
        self._fill_lane_msg(self.middle_lane_wpnts_msg, middle_lane_resampled, middle_s, middle_d)
        self._fill_lane_msg(self.inner_lane_wpnts_msg, inner_lane_resampled, inner_s, inner_d)

        # ------------- Plot (only when vis:=true) -------------
        if self.vis:
            plt.figure(figsize=(8, 6))
            plt.plot(original_center_wpnts[:, 0], original_center_wpnts[:, 1], 'k-', label='Center Line')
            plt.plot(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1], 'g--', label='Outer Lane')
            plt.plot(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1], 'b--', label='Inner Lane')

            plt.scatter(original_center_wpnts[:, 0], original_center_wpnts[:, 1], c='k', s=10)
            plt.scatter(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1], c='g', s=10)
            plt.scatter(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1], c='b', s=10)

            plt.axis('equal')
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title('Lane Visualization')
            plt.legend()
            plt.grid(True)
            plt.show()
        # ------------- Plot -------------

        # Cache the polylines so the lane-marker timer can republish them at a fixed rate,
        # independent of the vis flag and without needing an rqt/lane_offset change.
        self._center_xy = original_center_wpnts
        self._outer_xy = outer_lane_resampled
        self._inner_xy = inner_lane_resampled

    def publish_start_end_markers(self, start_s, obs_start_u, obs_end_u, end_s):
        """Publish the four key s points of the avoidance span as large sphere markers:
        path start (= car, cyan), obstacle start (yellow), obstacle end (orange),
        path end (magenta). All are placed on the raceline (d=0)."""
        if self.avoidance_pts_pub.get_subscription_count() == 0:  # rviz-off: skip marker
            return
        pts_s = np.array([start_s, obs_start_u, obs_end_u, end_s]) % self.scaled_max_s
        try:
            resp = self.converter.get_cartesian(pts_s, np.zeros_like(pts_s))
        except Exception:
            return
        resp = np.asarray(resp, dtype=float)
        xy = resp.T if resp.shape[0] == 2 else resp
        colors = [(0.0, 1.0, 1.0), (1.0, 1.0, 0.0), (1.0, 0.5, 0.0), (1.0, 0.0, 1.0)]
        # Fixed ids, overwritten in place so the spheres sit steady; cleared by clear_all_markers().
        mrks = MarkerArray()
        for i in range(min(len(xy), 4)):
            mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            mrk.ns = "avoidance_start_end"
            mrk.id = i
            mrk.type = Marker.SPHERE
            mrk.action = Marker.ADD
            mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.4
            mrk.color.a = 0.9
            mrk.color.r, mrk.color.g, mrk.color.b = colors[i]
            mrk.pose.position.x = float(xy[i][0])
            mrk.pose.position.y = float(xy[i][1])
            mrk.pose.position.z = 0.2
            mrk.pose.orientation.w = 1.0
            mrks.markers.append(mrk)
        self.avoidance_pts_pub.publish(mrks)

    def lane_viz_loop(self):
        """Fixed-rate republish of the cached lane markers and current offset."""
        if self._center_xy is None:
            return
        self.publish_lane_markers(self._center_xy, self._outer_xy, self._inner_xy)
        self.lane_offset_pub.publish(Float32MultiArray(data=[float(self.lane_offset), 0.0, float(-self.lane_offset)]))

    def publish_lane_markers(self, center_xy, outer_xy, inner_xy):
        """Publish center / outer(left) / inner(right) lanes as colored line strips.
        Center: white, outer/left: green, inner/right: red.
        While an evasion side is committed, the CHOSEN lane is drawn thick + yellow and a
        text label 'OT LEFT/MIDDLE/RIGHT' floats over the car, so the choice is visible live.
        The white center strip doubles as the middle lane (same geometry)."""
        if self.lane_mrks_pub.get_subscription_count() == 0:  # rviz-off: skip marker
            return
        # committed_side is set by _apply_side_hysteresis during evasions and reset to None
        # by the debounced clear in loop() when overtaking stops.
        chosen_ns = {"left": "outer_left", "middle": "center", "right": "inner_right"}.get(self.committed_side)
        mrks = MarkerArray()
        specs = [
            ("center", center_xy, (1.0, 1.0, 1.0)),
            ("outer_left", outer_xy, (0.1, 0.9, 0.1)),
            ("inner_right", inner_xy, (0.9, 0.1, 0.1)),
        ]
        for mid, (ns, xy, (r, g, b)) in enumerate(specs):
            mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            mrk.ns = ns
            mrk.id = mid
            mrk.type = Marker.LINE_STRIP
            mrk.action = Marker.ADD
            if ns == chosen_ns:
                mrk.scale.x = 0.10
                r, g, b = 1.0, 0.85, 0.1
            else:
                mrk.scale.x = 0.03
            mrk.color.a = 1.0
            mrk.color.r, mrk.color.g, mrk.color.b = r, g, b
            mrk.pose.orientation.w = 1.0
            mrk.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in xy]
            mrks.markers.append(mrk)

        # Floating side label at the car. Published with a fixed ns/id so it overwrites in
        # place; explicitly DELETEd when no side is committed (no lifetime -> no flicker).
        txt = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
        txt.ns = "chosen_side"
        txt.id = 0
        if chosen_ns is not None and self.current_s is not None and self.converter is not None:
            xy = np.asarray(self.converter.get_cartesian(
                np.array([self.current_s % self.scaled_max_s]), np.array([self.current_d])), dtype=float)
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.text = f"OT {self.committed_side.upper()}"
            txt.scale.z = 0.5
            txt.color.a = 1.0
            txt.color.r, txt.color.g, txt.color.b = 1.0, 0.85, 0.1
            txt.pose.position.x = float(xy[0][0])
            txt.pose.position.y = float(xy[1][0])
            txt.pose.position.z = 0.8
            txt.pose.orientation.w = 1.0
        else:
            txt.action = Marker.DELETE
        mrks.markers.append(txt)

        self.lane_mrks_pub.publish(mrks)

    def clear_all_markers(self):
        """Send DELETEALL to every avoidance marker topic so nothing lingers in rviz when
        we stop overtaking. Also publishes an empty evasion path to reset the merger side."""
        for pub in (self.mrks_pub, self.spline_sample_pub, self.avoidance_pts_pub):
            if pub.get_subscription_count() == 0:  # rviz-off: skip marker
                continue
            mrks = MarkerArray()
            del_mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)
            pub.publish(mrks)
        self.evasion_pub.publish(
            OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"), wpnts=[])
        )

    ### Main Loop ###
    def loop(self):
        start_time = time.perf_counter()

        # Obstacle pre-processing
        obs = deepcopy(self.obs_perception)
        considered_obs = self.obstacle_preprocessing(obs=obs)

        # Overtake only with a fresh GP-trajectory prediction when require_prediction is set.
        evasion_ok = False
        if len(considered_obs) > 0 and (not self.require_prediction
                                        or (self.prediction_is_fresh() and not self.force_trailing)):
            evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.lane_change(considered_obs, self.current_s)
            # Publish merge reagion if evasion track has been found
            if len(evasion_s) > 0:
                evasion_ok = True
                self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end % self.scaled_max_s, evasion_s[-1] % self.scaled_max_s]))

        # Debounce clearing: only wipe markers after several consecutive idle frames so a single
        # failed frame does not flicker the markers off and back on.
        if evasion_ok:
            self.no_evasion_count = 0
        else:
            self.no_evasion_count += 1
            if self.no_evasion_count == self.clear_after_frames:
                self.clear_all_markers()
                self.committed_side = None
                self.pending_side = None
                self.pending_count = 0

        # publish latency
        if self.measure:
            self.measure_pub.publish(Float32(data=time.perf_counter() - start_time))


def main(args=None):
    rclpy.init(args=args)
    node = ChangeAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
