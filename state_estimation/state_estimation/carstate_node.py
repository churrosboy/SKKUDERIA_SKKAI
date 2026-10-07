import rclpy
from rclpy.node import Node
import tf2_ros

import time
import numpy as np

from sensor_msgs.msg import Imu
from f110_msgs.msg import WpntArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from frenet_conversion.frenet_converter import FrenetConverter
from tf_transformations import euler_from_quaternion

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import SetParametersResult
from stack_master.parameter_event_handler import ParameterEventHandler
from state_estimation.straight_blender import StraightBlender

# Carstate node is relevant for SE1 only (basic state estimaiton pipeline).
# The carstate node publishes the "final" state estimation of the car, in this case the velocities from the (early fusion) EKF and the pose from localization (by default cartographer SLAM).

class Carstate(Node):
    def __init__(self):
        super().__init__('carstate',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Carstate node started")

        # ros params
        self.declare_parameter('/carstate_node/odom_topic', "/early_fusion/odom")
        self.declare_parameter('/carstate_node/odom_out_topic', "/car_state/odom")
        self.declare_parameter('/carstate_node/pose_out_topic', "/car_state/pose")
        self.odom_in_topic = self.get_parameter('/carstate_node/odom_topic').value
        self.odom_out_topic = self.get_parameter('/carstate_node/odom_out_topic').value
        self.pose_out_topic = self.get_parameter('/carstate_node/pose_out_topic').value
        self.frenet_bool = self.get_parameter('frenet_bool').get_parameter_value().bool_value # Bool if we want frenet on or off

        # Wait until the requried tf transforms exist
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # data containers
        self.ekf_odom = None
        self.frenet_converter = None
        self.gb_wpnts = None
        self.car_state_odom = None

        # subscribers
        self.ekf_odom_sub = self.create_subscription(Odometry, self.odom_in_topic, self.ekf_odom_cb, 10)
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.gb_wpnts_cb, 10)

        # publishers
        self.state_odom_pub = self.create_publisher(Odometry, self.odom_out_topic, 10)
        self.state_pose_pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.frenet_state_odom_pub = self.create_publisher(Odometry, "/car_state/frenet/odom", 10)
        self.frenet_state_pose_pub = self.create_publisher(PoseStamped, "/car_state/frenet/pose", 10)

        for name, default in [
                ('straight_ekf_blend_enable', False),
                ('straight_scan_trust_alpha', 0.05),
                ('straight_reconverge_rate_mps', 0.5),
                ('straight_max_offset_m', 2.0),
                ('straight_reloc_jump_m', 1.0),
                ('straight_ekf_timeout_s', 0.3),
                ('straight_sector_node', 'sector_tuner')]:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self.blend_armed = (self.get_parameter('straight_ekf_blend_enable').value
                            and self.frenet_bool)
        self.blender = None
        self.frenet_cache = None
        self.last_blend_time = None
        self._sector_future = None
        self._n_sectors = None
        self._debug_decim = 0
        if self.blend_armed:
            sector_node = self.get_parameter('straight_sector_node').value
            self.sector_param_client = self.create_client(
                GetParameters, f'/{sector_node}/get_parameters')
            self.sector_init_timer = self.create_timer(1.0, self._sector_init_tick)
            self.param_handler = ParameterEventHandler(self)
            self.blend_debug_pub = self.create_publisher(
                PointStamped, '/car_state/straight_blend/debug', 10)
            self.add_on_set_parameters_callback(self._on_set_params)
            self.get_logger().info('[straight-blend] armed, waiting for sector params')

        # Block until relevant data is here
        self.wait_for_messages(frenet_bool=self.frenet_bool)

        # Publish at 80 Hz
        self.create_timer(1/80, self.cartesian_state_loop)
        if self.frenet_bool:
            self.create_timer(1/80, self.frenet_state_loop)

    def ekf_odom_cb(self, data):
        self.ekf_odom = data

    def gb_wpnts_cb(self, data):
        self.gb_wpnts = data

    def get_slam_tf(self) -> TransformStamped:
        trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        return trans


    def wait_for_messages(self, frenet_bool: bool = False):
        self.get_logger().info('Carstate Node waiting for Odometry messages...')
        ekf_print = False
        frenet_print = False
        while self.ekf_odom is None or (frenet_bool and self.gb_wpnts is None):
            rclpy.spin_once(self)
            if self.ekf_odom is not None and not ekf_print:
                self.get_logger().info('Received Odometry message.')
                ekf_print = True
            if frenet_bool and self.gb_wpnts is not None and not frenet_print:
                waypoint_array = self.gb_wpnts.wpnts
                waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
                waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
                waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
                self.frenet_converter = FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))
                self.get_logger().info('Received Global Waypoints message and frenet converter initialized!')
                frenet_print = True
        self.get_logger().info('All required messages received. Continuing...')


    def _sector_init_tick(self):
        """1 Hz non-blocking fetch of sector params from /sector_tuner.

        Never wait_for_service: carstate must not stall if sector_tuner is
        late or absent. Two-step: n_sectors first, then the per-sector triplet.
        """
        if not self.sector_param_client.service_is_ready():
            return
        if self._sector_future is not None:
            if not self._sector_future.done():
                return
            resp = self._sector_future.result()
            self._sector_future = None
            if resp is None:
                return
            if self._n_sectors is None:
                self._n_sectors = resp.values[0].integer_value
            else:
                self._apply_sector_response(resp)
            return
        req = GetParameters.Request()
        if self._n_sectors is None:
            req.names = ['n_sectors']
        else:
            names = []
            for i in range(self._n_sectors):
                names += [f'Sector{i}.start', f'Sector{i}.end', f'Sector{i}.is_straight']
            req.names = names
        self._sector_future = self.sector_param_client.call_async(req)

    def _apply_sector_response(self, resp):
        self.sectors = []
        for i in range(self._n_sectors):
            self.sectors.append({
                'start': resp.values[3 * i].integer_value,
                'end': resp.values[3 * i + 1].integer_value,
                'is_straight': resp.values[3 * i + 2].bool_value,
            })
        if self.frenet_converter is None:
            return
        self.blender = StraightBlender(
            track_length=self.frenet_converter.raceline_length,
            alpha=self.get_parameter('straight_scan_trust_alpha').value,
            reconverge_rate_mps=self.get_parameter('straight_reconverge_rate_mps').value,
            max_offset_m=self.get_parameter('straight_max_offset_m').value,
            reloc_jump_m=self.get_parameter('straight_reloc_jump_m').value)
        self._rebuild_straight_ranges()
        self.sector_init_timer.cancel()
        sector_node = self.get_parameter('straight_sector_node').value
        for i in range(self._n_sectors):
            self.param_handler.add_parameter_callback(
                f'Sector{i}.is_straight', sector_node, callback=self._is_straight_cb)
        self.get_logger().info(
            f'[straight-blend] active, straight ranges [m]: {self.blender.straight_ranges}')

    def _rebuild_straight_ranges(self):
        ranges = [(s['start'] * 0.1, (s['end'] + 1) * 0.1)
                  for s in self.sectors if s['is_straight']]
        self.blender.set_straight_ranges(ranges)

    def _is_straight_cb(self, p):
        idx = int(p.name.split('.')[0].replace('Sector', ''))
        self.sectors[idx]['is_straight'] = p.value.bool_value
        self._rebuild_straight_ranges()
        self.get_logger().info(
            f'[straight-blend] {p.name} -> {p.value.bool_value}, ranges now {self.blender.straight_ranges}')

    def _on_set_params(self, params):
        for p in params:
            if self.blender is None:
                break
            if p.name == 'straight_scan_trust_alpha':
                self.blender.alpha = float(p.value)
            elif p.name == 'straight_reconverge_rate_mps':
                self.blender.reconverge_rate_mps = float(p.value)
            elif p.name == 'straight_max_offset_m':
                self.blender.max_offset_m = float(p.value)
            elif p.name == 'straight_reloc_jump_m':
                self.blender.reloc_jump_m = float(p.value)
        return SetParametersResult(successful=True)

    def cartesian_state_loop(self):
        # publish SLAM cartesian positional and velocity data
        carstate_pose_msg = PoseStamped()
        carstate_odom_msg = Odometry()

        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        except Exception as e:
            self.get_logger().warn(f"{e}")
            return

        x_pub = trans.transform.translation.x
        y_pub = trans.transform.translation.y

        if self.blender is not None:
            x_raw, y_raw = x_pub, y_pub
            q = trans.transform.rotation
            theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            frenet_pos = self.frenet_converter.get_frenet([x_raw], [y_raw])
            s_raw = frenet_pos[0, 0]
            d_raw = frenet_pos[1, 0]
            frenet_vel = self.frenet_converter.get_frenet_velocities(
                self.ekf_odom.twist.twist.linear.x, self.ekf_odom.twist.twist.linear.y, theta)
            vs = float(frenet_vel[0][0])
            vd = float(frenet_vel[1][0])

            now = self.get_clock().now()
            ekf_age = (now - rclpy.time.Time.from_msg(self.ekf_odom.header.stamp)).nanoseconds * 1e-9
            dt = ((now.nanoseconds - self.last_blend_time) * 1e-9
                  if self.last_blend_time is not None else -1.0)
            self.last_blend_time = now.nanoseconds

            if ekf_age > self.get_parameter('straight_ekf_timeout_s').value:
                self.blender.reset(s_raw)
                s_out, offset, mode = s_raw, 0.0, StraightBlender.MODE_PASSTHROUGH
            else:
                s_out, offset, mode = self.blender.update(s_raw, vs, dt)

            if offset != 0.0:
                xy = self.frenet_converter.get_cartesian(
                    s_out % self.frenet_converter.raceline_length, d_raw)
                x_pub, y_pub = float(xy[0]), float(xy[1])
            self.frenet_cache = (trans.header.stamp, s_out, d_raw, vs, vd)

            self._debug_decim += 1
            if self._debug_decim >= 8:
                self._debug_decim = 0
                dbg = PointStamped()
                dbg.header.stamp = trans.header.stamp
                dbg.header.frame_id = 'frenet'
                dbg.point.x = offset
                dbg.point.y = float(mode)
                dbg.point.z = s_out
                self.blend_debug_pub.publish(dbg)

        # build pose message
        carstate_pose_msg.header = trans.header
        carstate_pose_msg.pose.position.x = x_pub
        carstate_pose_msg.pose.position.y = y_pub
        carstate_pose_msg.pose.position.z = trans.transform.translation.z
        carstate_pose_msg.pose.orientation = trans.transform.rotation

        # build odometry message
        carstate_odom_msg.header = carstate_pose_msg.header
        carstate_odom_msg.pose.pose = carstate_pose_msg.pose

        # handle speed from EKF
        carstate_odom_msg.twist.twist = self.ekf_odom.twist.twist  # Make sure to use twist.twist
        self.car_state_odom = carstate_odom_msg

        # publish
        self.state_odom_pub.publish(carstate_odom_msg)
        self.state_pose_pub.publish(carstate_pose_msg)


    def frenet_state_loop(self):
        if self.car_state_odom is None:
            return

        if self.frenet_cache is not None and self.frenet_cache[0] == self.car_state_odom.header.stamp:
            stamp, s, d, vs, vd = self.frenet_cache
            frenet_pose_msg = PoseStamped()
            frenet_pose_msg.header.stamp = stamp
            frenet_pose_msg.header.frame_id = "frenet"
            frenet_pose_msg.pose.position.x = s
            frenet_pose_msg.pose.position.y = d
            frenet_odom_msg = Odometry()
            frenet_odom_msg.header = frenet_pose_msg.header
            frenet_odom_msg.pose.pose = frenet_pose_msg.pose
            frenet_odom_msg.twist.twist.linear.x = vs
            frenet_odom_msg.twist.twist.linear.y = vd
            self.frenet_state_odom_pub.publish(frenet_odom_msg)
            self.frenet_state_pose_pub.publish(frenet_pose_msg)
            return

        odom_cart = self.car_state_odom
        x_cart = odom_cart.pose.pose.position.x
        y_cart = odom_cart.pose.pose.position.y
        vx = odom_cart.twist.twist.linear.x
        vy = odom_cart.twist.twist.linear.y
        q_cart = odom_cart.pose.pose.orientation
        theta = euler_from_quaternion([q_cart.x, q_cart.y, q_cart.z, q_cart.w])[2]

        # get frenet coordinates and velocities
        frenet_pos = self.frenet_converter.get_frenet([x_cart], [y_cart])
        frenet_vel = self.frenet_converter.get_frenet_velocities(vx, vy, theta)

        s = frenet_pos[0, 0]
        d = frenet_pos[1, 0]
        vs = frenet_vel[0][0]
        vd = frenet_vel[1][0]

        #frenet pose msg
        frenet_pose_msg = PoseStamped()
        frenet_pose_msg.header.stamp = odom_cart.header.stamp
        frenet_pose_msg.header.frame_id = "frenet"
        frenet_pose_msg.pose.position.x = s
        frenet_pose_msg.pose.position.y = d

        #frenet odom msg
        frenet_odom_msg = Odometry()
        frenet_odom_msg.header = frenet_pose_msg.header
        frenet_odom_msg.pose.pose = frenet_pose_msg.pose
        frenet_odom_msg.twist.twist.linear.x = vs
        frenet_odom_msg.twist.twist.linear.y = vd
        #TODO handle speed from EKF

        #publish
        self.frenet_state_odom_pub.publish(frenet_odom_msg)
        self.frenet_state_pose_pub.publish(frenet_pose_msg)

def main():
    rclpy.init()
    node = Carstate()
    rclpy.spin(node)
    rclpy.shutdown()
