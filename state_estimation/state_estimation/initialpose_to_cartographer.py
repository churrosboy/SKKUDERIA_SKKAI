#!/usr/bin/env python3
"""Bridge RViz's "2D Pose Estimate" button to Cartographer pure localization.

Cartographer has no /initialpose subscriber (that topic is an AMCL/nav2
convention), so clicking the button normally does nothing on the car. This
node listens on /initialpose and, per click:

  1. finds all ACTIVE trajectories via /get_trajectory_states
  2. finishes them via /finish_trajectory
  3. starts a fresh localization trajectory at the clicked pose via
     /start_trajectory (relative to trajectory 0, the frozen .pbstream map)

Result: localization restarts instantly at the clicked pose instead of
waiting for Cartographer's sampled global relocalization to converge.

Snap stage: the RViz click is only a rough seed (hand-placed, roughly
10-30 cm / 10 deg off). Before restarting the trajectory, the latest /scan is
matched against the /map occupancy grid (map_server) in a small window around
the click (coarse-to-fine correlative search on a likelihood field built from
the wall surface, see scan_matcher.py), and the trajectory is started at the
refined pose. Cartographer's local scan matcher only matches against its own
freshly built submap, never against the frozen map, so without this step a
slightly-off click is held as-is until a sampled pose-graph constraint lands.
A missing base_link->laser TF skips the snap and uses the raw click instead
(see lookup_laser_tf). Kill switch: snap_enable:=False restores the raw-click
behavior.

Hand-guided collision recovery (collision_reloc_enable, default OFF;
independent of snap_enable -- the /map field is built for either and the
recovery always snaps its own restarts while RViz clicks stay raw if
snap_enable is off). After a crash the car can only be pushed back onto the
track. collision_stop (C++) latches when its reverse does not free the car and
publishes the pre-crash map pose on collision_stop/prior_pose. This node then:
  1. relocalizes at that prior right away (snap +-0.5 m / +-15 deg), so the
     map pose is "where the crash happened" before anyone touches the car;
  2. waits for the put-down: /scan changes (the push), then holds still for
     putdown_still_sec (odometry is useless here -- motor-off erpm ~ 0);
  3. searches along the raceline within +-reloc_s_window m of the crash s,
     several lateral offsets and headings incl. reversed
     (scan_matcher.raceline_search), refines with the usual snap, checks the
     result is inside the track bounds and not ambiguous;
  4. restarts the trajectory there and publishes collision_stop/release so
     collision_stop hands control back to navigation.
Aborts whenever /collision_detected drops (A-button reset, manual mode).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from f110_msgs.msg import WpntArray
from cartographer_ros_msgs.msg import StatusCode, TrajectoryStates
from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    StartTrajectory,
)

# matching maths live in scan_matcher.py (pure numpy, unit tested offline)
from state_estimation.scan_matcher import (
    LikelihoodField,
    correlative_match,
    distance_transform_edt,
    nearest_waypoint,
    raceline_search,
    signed_lateral,
)

# The trajectory loaded from the .pbstream (-load_state_filename) is always
# id 0 and FROZEN; initial poses are expressed relative to it.
MAP_TRAJECTORY_ID = 0
SERVICE_TIMEOUT_SEC = 5.0


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class InitialPoseToCartographer(Node):

    def __init__(self):
        super().__init__('initialpose_to_cartographer')

        self.declare_parameter('configuration_directory', '')
        self.declare_parameter('configuration_basename', 'f110_2d_loc.lua')
        self.config_dir = self.get_parameter('configuration_directory').value
        self.config_basename = self.get_parameter('configuration_basename').value
        if not self.config_dir:
            raise RuntimeError(
                'configuration_directory param is required (path to the dir '
                'containing the localization .lua)')

        # --- scan-to-map snap ---
        self.declare_parameter('snap_enable', True)
        self.declare_parameter('snap_search_xy', 0.5)        # m, +-
        self.declare_parameter('snap_search_yaw_deg', 15.0)  # deg, +-
        self.declare_parameter('snap_yaw_step_deg', 1.0)
        self.declare_parameter('snap_fine_yaw_step_deg', 0.2)
        self.declare_parameter('snap_sigma', 0.10)           # m, field width
        self.declare_parameter('snap_min_score', 0.35)       # reject below
        self.declare_parameter('snap_max_beams', 400)
        self.declare_parameter('snap_scan_max_age', 1.0)     # s
        self.declare_parameter('base_frame', 'base_link')
        gp = self.get_parameter
        self.snap_enable = gp('snap_enable').value
        self.snap_search_xy = gp('snap_search_xy').value
        self.snap_search_yaw = math.radians(gp('snap_search_yaw_deg').value)
        self.snap_yaw_step = math.radians(gp('snap_yaw_step_deg').value)
        self.snap_fine_yaw_step = math.radians(
            gp('snap_fine_yaw_step_deg').value)
        self.snap_sigma = gp('snap_sigma').value
        self.snap_min_score = gp('snap_min_score').value
        self.snap_max_beams = gp('snap_max_beams').value
        self.snap_scan_max_age = gp('snap_scan_max_age').value
        self.base_frame = gp('base_frame').value

        # --- restart scan gate ---
        # wait for a /scan stamped after finish_trajectory before start_trajectory
        self.declare_parameter('restart_scan_gate', True)
        self.declare_parameter('restart_gate_margin_sec', 0.1)
        self.declare_parameter('restart_gate_timeout_sec', 3.0)
        self.restart_scan_gate = gp('restart_scan_gate').value
        self.restart_gate_margin = gp('restart_gate_margin_sec').value
        self.restart_gate_timeout = gp('restart_gate_timeout_sec').value

        # --- startup pose ---
        # one self-applied relocalization after startup_delay_sec (default OFF)
        self.declare_parameter('startup_pose_enable', False)
        self.declare_parameter('startup_pose', [0.0])   # [x, y, yaw]; [0.0] = unset
        self.declare_parameter('startup_map_yaml', '')
        self.declare_parameter('startup_delay_sec', 3.0)
        self.startup_pose_enable = gp('startup_pose_enable').value
        self.startup_pose = list(gp('startup_pose').value)
        self.startup_map_yaml = gp('startup_map_yaml').value
        self.startup_delay = gp('startup_delay_sec').value

        # --- hand-guided collision recovery (see docstring) ---
        self.declare_parameter('collision_reloc_enable', False)
        self.declare_parameter('putdown_still_sec', 1.0)      # s still after push
        self.declare_parameter('putdown_move_thresh', 0.05)   # m mean |dr| = moving
        self.declare_parameter('putdown_timeout_sec', 120.0)  # give up waiting
        self.declare_parameter('putdown_poll_hz', 5.0)
        self.declare_parameter('reloc_s_window', 4.0)         # m +- along track
        self.declare_parameter('reloc_d_offsets', [-0.6, -0.3, 0.0, 0.3, 0.6])
        self.declare_parameter('reloc_yaw_deg', 45.0)         # +- around raceline psi
        self.declare_parameter('reloc_yaw_step_deg', 5.0)
        self.declare_parameter('reloc_allow_reverse', True)   # set down backwards
        self.declare_parameter('reloc_prior_weight', 0.02)    # score penalty / m of s
        self.declare_parameter('reloc_min_margin', 0.05)      # best - runner-up(>1 m)
        self.declare_parameter('reloc_bound_margin', 0.3)     # m outside d_left/right
        self.declare_parameter('reloc_max_beams', 120)
        # wider snap window for the crash site (the car is stationary while latched)
        self.declare_parameter('reloc_prior_search_xy', 1.0)   # m +-
        # joystick button that forces a snap attempt while waiting for the put-down
        self.declare_parameter('reloc_trigger_button', 3)      # Y
        self.collision_reloc_enable = gp('collision_reloc_enable').value
        self.reloc_prior_search_xy = gp('reloc_prior_search_xy').value
        self.reloc_trigger_button = gp('reloc_trigger_button').value
        self.putdown_still_sec = gp('putdown_still_sec').value
        self.putdown_move_thresh = gp('putdown_move_thresh').value
        self.putdown_timeout = gp('putdown_timeout_sec').value
        self.putdown_poll_hz = gp('putdown_poll_hz').value
        self.reloc_s_window = gp('reloc_s_window').value
        self.reloc_d_offsets = list(gp('reloc_d_offsets').value)
        self.reloc_yaw = math.radians(gp('reloc_yaw_deg').value)
        self.reloc_yaw_step = math.radians(gp('reloc_yaw_step_deg').value)
        self.reloc_allow_reverse = gp('reloc_allow_reverse').value
        self.reloc_prior_weight = gp('reloc_prior_weight').value
        self.reloc_min_margin = gp('reloc_min_margin').value
        self.reloc_bound_margin = gp('reloc_bound_margin').value
        self.reloc_max_beams = gp('reloc_max_beams').value
        self.wpnts = None          # (M,4) [x, y, psi, s]
        self.wpnt_bounds = None    # (M,2) [d_left, d_right]
        self.collision_active = False
        self.collision_time = None

        self.lf = None           # LikelihoodField, built on /map
        self.latest_scan = None
        self.laser_tf = None     # (x, y, yaw) base_link -> laser, cached
        # the /map field is also needed by the collision recovery (click snap may be off)
        if self.snap_enable or self.collision_reloc_enable:
            # base_link->laser is static: lookup_laser_tf opens a temporary listener on first use
            self.tf_buffer = None
            self.tf_listener = None
            # map_server publishes /map latched (TRANSIENT_LOCAL)
            map_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                OccupancyGrid, '/map', self.map_cb, map_qos)
            # /scan is grabbed on demand by a temporary subscription (grab_scan)
            # refined pose, for checking in RViz
            self.snap_pub = self.create_publisher(
                PoseStamped, '/initialpose_snapped', 1)
            if distance_transform_edt is None:
                self.get_logger().warn(
                    'scipy not available: snap likelihood field degrades '
                    'to a hard occupied/free mask')

        # Reentrant group + MultiThreadedExecutor so the synchronous service
        # calls inside the subscription callback can be serviced.
        self.cb_group = ReentrantCallbackGroup()
        self.get_states_client = self.create_client(
            GetTrajectoryStates, 'get_trajectory_states',
            callback_group=self.cb_group)
        self.finish_client = self.create_client(
            FinishTrajectory, 'finish_trajectory',
            callback_group=self.cb_group)
        self.start_client = self.create_client(
            StartTrajectory, 'start_trajectory',
            callback_group=self.cb_group)

        self.busy = False
        # RViz publishes /initialpose BEST_EFFORT; a best-effort subscription
        # accepts both publisher types
        initialpose_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.initialpose_cb, initialpose_qos,
            callback_group=self.cb_group)

        self.get_logger().info(
            'Relaying /initialpose to Cartographer '
            f'({self.config_dir}/{self.config_basename}), '
            f'scan-to-map snap {"ON" if self.snap_enable else "OFF"}')

        if self.startup_pose_enable:
            self.startup_timer = self.create_timer(
                self.startup_delay, self.startup_pose_cb,
                callback_group=self.cb_group)

        if self.collision_reloc_enable:
            # /global_waypoints is republished every 10 s; a permanent subscription is cheap
            self.create_subscription(
                WpntArray, '/global_waypoints', self.wpnts_cb, 1)
            self.create_subscription(
                Bool, '/collision_detected', self.collision_cb, 1)
            self.create_subscription(
                PoseStamped, '/collision_stop/prior_pose',
                self.prior_pose_cb, 1, callback_group=self.cb_group)
            self.release_pub = self.create_publisher(
                Bool, '/collision_stop/release', 1)
            self.get_logger().info(
                'collision recovery ARMED: waiting for '
                f'/collision_stop/prior_pose (RViz click snap '
                f'{"on" if self.snap_enable else "OFF"})')

    # ------------------------------------------------------------------
    # hand-guided collision recovery
    # ------------------------------------------------------------------
    def wpnts_cb(self, msg: WpntArray):
        if not msg.wpnts:
            return
        w = np.array([[p.x_m, p.y_m, p.psi_rad, p.s_m] for p in msg.wpnts])
        self.wpnts = w
        self.wpnt_bounds = np.array([[p.d_left, p.d_right] for p in msg.wpnts])

    def collision_cb(self, msg: Bool):
        self.collision_active = msg.data
        self.collision_time = time.monotonic()

    def collision_ongoing(self):
        """collision_stop still holds the car (True at 50 Hz while
        braking/reversing/latched). Stale = node died -> treat as over."""
        return (self.collision_active and self.collision_time is not None
                and time.monotonic() - self.collision_time < 0.5)

    def prior_pose_cb(self, msg: PoseStamped):
        if self.busy:
            self.get_logger().warn(
                'collision recovery: relocalization already running, '
                'ignoring prior')
            return
        self.busy = True
        try:
            self.collision_recovery(msg)
        except Exception as e:      # never let the recovery kill the node
            self.get_logger().error(f'collision recovery failed: {e!r}')
        finally:
            self.busy = False

    @staticmethod
    def _pose_msg(x, y, yaw, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg

    def collision_recovery(self, prior: PoseStamped):
        px, py = prior.pose.position.x, prior.pose.position.y
        pyaw = yaw_from_quat(prior.pose.orientation)
        self.get_logger().warn(
            f'collision recovery: prior ({px:.2f}, {py:.2f}, '
            f'{math.degrees(pyaw):.0f} deg) — relocalizing there, then '
            'waiting for the car to be pushed back on track')
        # 1. reset the map pose to the crash site before anyone touches the
        #    car (the spinning wheels dragged it off through odometry)
        self.relocalize(self._pose_msg(
            px, py, pyaw, self.get_clock().now().to_msg()), snap=True,
            search_xy=self.reloc_prior_search_xy)
        if self.wpnts is None:
            self.get_logger().error(
                'collision recovery: no /global_waypoints yet, cannot '
                'search along the raceline; leaving the car latched')
            return
        i0 = nearest_waypoint(self.wpnts[:, :2], px, py)
        s_center = float(self.wpnts[i0, 3])
        t_end = time.monotonic() + self.putdown_timeout
        need_move = True
        while time.monotonic() < t_end:
            # 2. put-down: scan changes (push) then holds still
            if not self.wait_for_putdown(t_end, need_move):
                return
            # 3. where is it now?
            found = self.raceline_relocalize(s_center)
            if found is None:
                need_move = False   # re-score after the next still period
                self.get_logger().warn(
                    'collision recovery: no confident pose yet — nudge the '
                    'car / wait; retrying')
                continue
            x, y, yaw = found
            # 4. restart the trajectory there and release collision_stop
            self.relocalize(self._pose_msg(
                x, y, yaw, self.get_clock().now().to_msg()), snap=True)
            self.release_pub.publish(Bool(data=True))
            self.get_logger().warn(
                f'collision recovery: relocalized at ({x:.2f}, {y:.2f}, '
                f'{math.degrees(yaw):.0f} deg), released collision_stop')
            return
        self.get_logger().error(
            f'collision recovery: gave up after {self.putdown_timeout:.0f} s '
            '(press the reset button)')

    @staticmethod
    def _scan_delta(a: LaserScan, b: LaserScan, n=100):
        ra = np.asarray(a.ranges, dtype=np.float64)
        rb = np.asarray(b.ranges, dtype=np.float64)
        if len(ra) != len(rb) or len(ra) == 0:
            return float('inf')
        idx = np.linspace(0, len(ra) - 1, min(n, len(ra))).astype(int)
        ra, rb = ra[idx], rb[idx]
        ok = np.isfinite(ra) & np.isfinite(rb) & (ra > a.range_min) & (rb > b.range_min)
        if ok.sum() < 10:
            return float('inf')
        return float(np.mean(np.abs(ra[ok] - rb[ok])))

    def wait_for_putdown(self, t_end, need_move):
        """Block until the scan has changed (the push; skipped when
        need_move is False) and then stayed still for putdown_still_sec.
        Polls /scan at putdown_poll_hz with temporary subscriptions (no
        40 Hz deserialization for minutes). Returns False on abort
        (collision over: A-button / manual mode) or timeout."""
        period = 1.0 / max(self.putdown_poll_hz, 0.5)
        prev = None
        moved = not need_move
        still_since = None
        # one temporary /scan and one temporary /joy subscription for the whole
        # wait; the loop only reads the latest messages
        latest = {'scan': None, 'btn': False, 'btn_prev': False}

        def scan_cb(m):
            latest['scan'] = m

        def joy_cb(m):
            b = self.reloc_trigger_button
            pressed = 0 <= b < len(m.buttons) and m.buttons[b] == 1
            if pressed and not latest['btn_prev']:
                latest['btn'] = True        # rising edge, consumed below
            latest['btn_prev'] = pressed

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        scan_sub = self.create_subscription(
            LaserScan, '/scan', scan_cb, qos, callback_group=self.cb_group)
        joy_sub = self.create_subscription(
            Joy, '/joy', joy_cb, 1, callback_group=self.cb_group)
        try:
            while time.monotonic() < t_end:
                if not self.collision_ongoing():
                    self.get_logger().warn(
                        'collision recovery: collision_stop released by the '
                        'user — aborting')
                    return False
                if latest['btn']:
                    latest['btn'] = False
                    self.get_logger().warn(
                        f'collision recovery: button {self.reloc_trigger_button} '
                        '— snapping now')
                    return True
                scan = latest['scan']
                if scan is not None and prev is not None and scan is not prev:
                    d = self._scan_delta(prev, scan)
                    self.get_logger().debug(
                        f'collision recovery: scan delta {d:.3f} m '
                        f'(moved={moved}, still='
                        f'{0.0 if still_since is None else time.monotonic() - still_since:.1f} s)')
                    if d > self.putdown_move_thresh:
                        if not moved:
                            self.get_logger().info(
                                'collision recovery: car is being moved')
                        moved = True
                        still_since = None
                    elif moved:
                        if still_since is None:
                            still_since = time.monotonic()
                        elif time.monotonic() - still_since >= self.putdown_still_sec:
                            self.get_logger().info(
                                'collision recovery: car put down and still')
                            return True
                if scan is not None:
                    prev = scan
                time.sleep(period)
            return False
        finally:
            self.destroy_subscription(scan_sub)
            self.destroy_subscription(joy_sub)

    def raceline_relocalize(self, s_center):
        """Coarse raceline-window search + fine snap + gates.
        Returns (x, y, yaw) or None."""
        scan = self.grab_scan(self.snap_scan_max_age)
        if scan is None or self.lf is None:
            return None
        pts = self.scan_points_base(scan)
        if pts is None:
            return None
        yaw_offs = np.arange(-self.reloc_yaw, self.reloc_yaw + 1e-9,
                             self.reloc_yaw_step)
        t0 = time.monotonic()
        x, y, yaw, score, second, sdist = raceline_search(
            self.lf, pts, self.wpnts, s_center, self.reloc_s_window,
            self.reloc_d_offsets, yaw_offs, self.reloc_allow_reverse,
            max_beams=self.reloc_max_beams,
            prior_weight=self.reloc_prior_weight)
        fx, fy, fyaw, fscore, _ = correlative_match(
            self.lf, pts, (x, y, yaw), self.snap_search_xy,
            self.snap_search_yaw, self.snap_yaw_step, self.snap_fine_yaw_step)
        dt = 1e3 * (time.monotonic() - t0)
        sec = (f'{second:.2f} @ {sdist:.1f} m' if second is not None else 'none')
        self.get_logger().info(
            f'collision recovery: raceline search {score:.2f} '
            f'(runner-up {sec}) -> snap {fscore:.2f} at ({fx:.2f}, {fy:.2f}, '
            f'{math.degrees(fyaw):.0f} deg) in {dt:.0f} ms')
        if fscore < self.snap_min_score:
            self.get_logger().warn(
                f'collision recovery: score {fscore:.2f} < snap_min_score '
                f'{self.snap_min_score}')
            return None
        if second is not None and score - second < self.reloc_min_margin:
            self.get_logger().warn(
                f'collision recovery: ambiguous — runner-up {sdist:.1f} m '
                f'away within {score - second:.2f} (< reloc_min_margin '
                f'{self.reloc_min_margin})')
            return None
        i = nearest_waypoint(self.wpnts[:, :2], fx, fy)
        d = signed_lateral(self.wpnts, i, fx, fy)
        d_left, d_right = self.wpnt_bounds[i]
        if not (-d_right - self.reloc_bound_margin <= d
                <= d_left + self.reloc_bound_margin):
            self.get_logger().warn(
                f'collision recovery: pose is outside the track '
                f'(d={d:+.2f}, bounds -{d_right:.2f}..+{d_left:.2f})')
            return None
        return (fx, fy, fyaw)

    # ------------------------------------------------------------------
    # startup pose
    # ------------------------------------------------------------------
    def resolve_startup_pose(self):
        """(x, y, yaw) from the startup_pose param or the map yaml, or None."""
        if len(self.startup_pose) == 3:
            return tuple(float(v) for v in self.startup_pose)
        if self.startup_map_yaml:
            try:
                import yaml
                with open(self.startup_map_yaml) as f:
                    ip = yaml.safe_load(f).get('initial_pose')
                if ip is not None and len(ip) >= 3:
                    return (float(ip[0]), float(ip[1]), float(ip[2]))
                self.get_logger().error(
                    f'startup pose: no initial_pose [x, y, yaw] in '
                    f'{self.startup_map_yaml}')
            except Exception as e:
                self.get_logger().error(
                    f'startup pose: cannot read {self.startup_map_yaml}: {e}')
            return None
        self.get_logger().error(
            'startup pose: neither startup_pose [x, y, yaw] nor '
            'startup_map_yaml set')
        return None

    def startup_pose_cb(self):
        self.startup_timer.cancel()   # one-shot
        pose = self.resolve_startup_pose()
        if pose is None:
            return
        x, y, yaw = pose
        # Wait for cartographer + (if snapping) /map and a live /scan so the
        # startup relocalization gets the same treatment as an RViz click.
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            ready = self.start_client.service_is_ready()
            if self.snap_enable:
                # only /map gates readiness; snap_pose grabs a live scan
                ready = ready and self.lf is not None
            if ready:
                break
            time.sleep(0.1)
        else:
            self.get_logger().warn(
                'startup pose: cartographer/map/scan not all ready after '
                f'{SERVICE_TIMEOUT_SEC:.0f} s; applying anyway')
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.get_logger().info(
            f'startup pose: relocalizing at x={x:.2f} y={y:.2f} '
            f'yaw={math.degrees(yaw):.1f} deg')
        self.initialpose_cb(msg)

    # ------------------------------------------------------------------
    # snap helpers
    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        info = msg.info
        oyaw = yaw_from_quat(info.origin.orientation)
        if abs(oyaw) > 1e-3:
            self.get_logger().warn(
                f'/map origin has yaw {math.degrees(oyaw):.1f} deg; snap '
                'assumes an axis-aligned map and will be off')
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            info.height, info.width)
        # correlative_match clips its window to pad-1 cells, so the padding
        # must cover the wider crash-site window too
        pad = int(round(max(self.snap_search_xy, self.reloc_prior_search_xy)
                        / info.resolution)) + 2
        t0 = time.monotonic()
        self.lf = LikelihoodField(
            grid, info.resolution,
            (info.origin.position.x, info.origin.position.y),
            self.snap_sigma, pad)
        self.get_logger().info(
            f'snap: likelihood field built from /map '
            f'{info.width}x{info.height} @ {info.resolution} m '
            f'in {1e3 * (time.monotonic() - t0):.0f} ms')

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def grab_scan(self, timeout):
        """Open a temporary BEST_EFFORT /scan subscription,
        wait up to `timeout` s for one message, destroy it and return the
        scan (or None). Keeps the node at zero /scan cost while idle."""
        got = {'msg': None}

        def cb(m):
            got['msg'] = m

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub = self.create_subscription(
            LaserScan, '/scan', cb, qos, callback_group=self.cb_group)
        t0 = time.monotonic()
        try:
            while got['msg'] is None and time.monotonic() - t0 < timeout:
                time.sleep(0.005)
        finally:
            self.destroy_subscription(sub)
        self.latest_scan = got['msg']
        return got['msg']

    def lookup_laser_tf(self, laser_frame, strict=True):
        """Cached base_link->laser transform. With strict=True a missing TF
        returns None instead of falling back to the identity: the TF tree may
        not be up yet, and silently assuming the laser sits at the base_link
        origin would shift every scan endpoint by the real mount offset,
        more than half the search window, and produce confidently wrong
        snaps that the score gate would accept."""
        if self.laser_tf is not None:
            return self.laser_tf
        # temporary listener, destroyed below so the node carries no /tf
        # subscription while idle
        import tf2_ros
        buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buf, self, spin_thread=False)
        try:
            tf = buf.lookup_transform(
                self.base_frame, laser_frame, Time(),
                timeout=Duration(seconds=0.5))
        except Exception as e:  # tf2 raises several exception types
            listener.unregister()
            if strict:
                self.get_logger().warn(
                    f'snap: no TF {self.base_frame}->{laser_frame} yet ({e}); '
                    'skipping snap, using raw click',
                    throttle_duration_sec=10.0)
                return None
            self.get_logger().warn(
                f'snap: no TF {self.base_frame}->{laser_frame} ({e}); '
                'assuming laser at base_link origin')
            return (0.0, 0.0, 0.0)
        listener.unregister()
        t = tf.transform.translation
        self.laser_tf = (t.x, t.y, yaw_from_quat(tf.transform.rotation))
        self.get_logger().info(
            f'snap: {self.base_frame}->{laser_frame} = '
            f'({t.x:.3f}, {t.y:.3f}, {math.degrees(self.laser_tf[2]):.1f} deg)')
        return self.laser_tf

    def scan_points_base(self, scan: LaserScan):
        """Valid scan endpoints as (N,2) in base_link, subsampled."""
        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        if ok.sum() < 20:
            return None
        r, ang = r[ok], ang[ok]
        if len(r) > self.snap_max_beams:
            idx = np.linspace(0, len(r) - 1, self.snap_max_beams).astype(int)
            r, ang = r[idx], ang[idx]
        laser_tf = self.lookup_laser_tf(scan.header.frame_id)
        if laser_tf is None:        # TF not up: raw click beats a 0.27 m error
            return None
        lx, ly, lyaw = laser_tf
        px = r * np.cos(ang)
        py = r * np.sin(ang)
        c, s = math.cos(lyaw), math.sin(lyaw)
        pts = np.empty((len(r), 2))
        pts[:, 0] = c * px - s * py + lx
        pts[:, 1] = s * px + c * py + ly
        return pts

    def snap_pose(self, pose, search_xy=None):
        """Refine the clicked pose against /map. Returns (x, y, yaw) or None
        when the snap cannot run / is rejected (caller keeps the raw click).
        search_xy: +- m window override (crash-site snap)."""
        if search_xy is None:
            search_xy = self.snap_search_xy
        if self.lf is None:
            self.get_logger().warn('snap: no /map received yet, using raw click')
            return None
        # grab a live scan now instead of caching at 40 Hz
        scan = self.grab_scan(self.snap_scan_max_age)
        if scan is None:
            self.get_logger().warn(
                f'snap: no /scan within {self.snap_scan_max_age:.1f} s, '
                'using raw click')
            return None
        age = (self.get_clock().now() - Time.from_msg(scan.header.stamp)
               ).nanoseconds * 1e-9
        if age > self.snap_scan_max_age:
            self.get_logger().warn(
                f'snap: latest /scan is {age:.2f} s old, using raw click')
            return None
        pts = self.scan_points_base(scan)
        if pts is None:
            # too few valid beams, or the laser TF is not up (both already
            # logged by scan_points_base / lookup_laser_tf)
            self.get_logger().warn('snap: no usable scan, using raw click')
            return None
        seed = (pose.position.x, pose.position.y,
                yaw_from_quat(pose.orientation))
        t0 = time.monotonic()
        x, y, yaw, score, seed_score = correlative_match(
            self.lf, pts, seed, search_xy, self.snap_search_yaw,
            self.snap_yaw_step, self.snap_fine_yaw_step)
        dt = 1e3 * (time.monotonic() - t0)
        self.get_logger().info(
            f'snap: score {seed_score:.2f} -> {score:.2f} in {dt:.0f} ms, '
            f'shift dx={x - seed[0]:+.3f} dy={y - seed[1]:+.3f} '
            f'dyaw={math.degrees(yaw - seed[2]):+.1f} deg '
            f'({len(pts)} beams)')
        if score < self.snap_min_score:
            self.get_logger().warn(
                f'snap: best score {score:.2f} < snap_min_score '
                f'{self.snap_min_score}, using raw click (is the car where '
                'you clicked? is /map the same map as the .pbstream?)')
            return None
        return (x, y, yaw)

    def call(self, client, request):
        """Synchronous service call with timeout; returns None on failure."""
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        # Executor spins this future on another thread of the pool.
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Service {client.srv_name} timed out')
                future.cancel()
                return None
            time.sleep(0.05)
        return future.result()

    def wait_for_fresh_scan(self, t_finish):
        """Block until a /scan stamped > t_finish + margin arrives (or timeout).

        Temporary BEST_EFFORT subscription; destroyed on return so the node
        carries no /scan cost outside a relocalization. Returns True if a
        fresh scan was seen, False on timeout (caller proceeds anyway).
        """
        gate = t_finish + Duration(seconds=self.restart_gate_margin)
        seen = {'stamp': None}

        def cb(m):
            seen['stamp'] = Time.from_msg(m.header.stamp)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub = self.create_subscription(
            LaserScan, '/scan', cb, qos, callback_group=self.cb_group)
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < self.restart_gate_timeout:
                st = seen['stamp']
                if st is not None and st > gate:
                    self.get_logger().info(
                        f'restart gate: fresh scan after '
                        f'{1e3 * (time.monotonic() - t0):.0f} ms '
                        f'(scan lag {(self.get_clock().now() - st).nanoseconds * 1e-9:.2f} s)')
                    return True
                time.sleep(0.01)
            self.get_logger().warn(
                f'restart gate: no /scan newer than finish time within '
                f'{self.restart_gate_timeout:.1f} s; starting anyway '
                '(cartographer may CHECK-fail on a stale scan)')
            return False
        finally:
            self.destroy_subscription(sub)

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        if self.busy:
            self.get_logger().warn(
                'Ignoring /initialpose: previous relocalization still running')
            return
        self.busy = True
        try:
            self.relocalize(msg)
        finally:
            self.busy = False

    def relocalize(self, msg: PoseWithCovarianceStamped, snap=None,
                   search_xy=None):
        # snap: None = follow snap_enable (RViz click path); True/False = force
        # search_xy: snap window override (crash-site snap)
        if snap is None:
            snap = self.snap_enable
        p = msg.pose.pose.position
        self.get_logger().info(
            f'Relocalizing at x={p.x:.2f} y={p.y:.2f} '
            f'(frame {msg.header.frame_id})')
        if msg.header.frame_id and msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'/initialpose frame is "{msg.header.frame_id}", expected '
                '"map" — check the RViz Fixed Frame; using the pose as-is')

        # 0. Refine the click against /map with the live scan. Runs BEFORE
        #    finish_trajectory: if it fails we still relocalize at the raw click.
        start_pose = msg.pose.pose
        if snap:
            snapped = self.snap_pose(msg.pose.pose, search_xy)
            if snapped is not None:
                x, y, yaw = snapped
                start_pose = PoseStamped().pose
                start_pose.position.x = x
                start_pose.position.y = y
                start_pose.position.z = 0.0
                qx, qy, qz, qw = quat_from_yaw(yaw)
                start_pose.orientation.x = qx
                start_pose.orientation.y = qy
                start_pose.orientation.z = qz
                start_pose.orientation.w = qw
                out = PoseStamped()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = 'map'
                out.pose = start_pose
                self.snap_pub.publish(out)

        # 1. Find and finish every ACTIVE trajectory (the frozen map
        #    trajectory 0 is not ACTIVE, so it is never touched).
        states = self.call(self.get_states_client,
                           GetTrajectoryStates.Request())
        if states is None:
            return
        active_ids = [
            tid for tid, tstate in zip(
                states.trajectory_states.trajectory_id,
                states.trajectory_states.trajectory_state)
            if tstate == TrajectoryStates.ACTIVE
        ]
        for tid in active_ids:
            req = FinishTrajectory.Request(trajectory_id=tid)
            resp = self.call(self.finish_client, req)
            if resp is None or resp.status.code != StatusCode.OK:
                detail = resp.status.message if resp else 'no response'
                self.get_logger().error(
                    f'finish_trajectory({tid}) failed: {detail}')
                return
            self.get_logger().info(f'Finished trajectory {tid}')

        # 1b. Don't start the new trajectory until /scan has caught up past the
        #     finish time (restart_scan_gate).
        if self.restart_scan_gate and active_ids:
            self.wait_for_fresh_scan(self.get_clock().now())

        # 2. Start a new localization trajectory at the clicked pose.
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_basename
        req.use_initial_pose = True
        req.initial_pose = start_pose
        req.relative_to_trajectory_id = MAP_TRAJECTORY_ID
        resp = self.call(self.start_client, req)
        if resp is None or resp.status.code != StatusCode.OK:
            detail = resp.status.message if resp else 'no response'
            self.get_logger().error(f'start_trajectory failed: {detail}')
            return
        sp = start_pose.position
        self.get_logger().info(
            f'Started trajectory {resp.trajectory_id} at x={sp.x:.2f} '
            f'y={sp.y:.2f} yaw={math.degrees(yaw_from_quat(start_pose.orientation)):.1f} deg'
            f'{" (snapped)" if start_pose is not msg.pose.pose else " (raw click)"}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseToCartographer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
