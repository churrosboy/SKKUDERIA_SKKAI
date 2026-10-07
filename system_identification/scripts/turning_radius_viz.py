#!/usr/bin/env python3
"""Fit a circle to the driven path and visualize it in RViz.

Drive the car in a steady circle with a constant steering command, e.g.:

    ros2 topic pub -r 20 /teleop ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {speed: 1.0, steering_angle: 0.3}}"

then run this script:

    python3 turning_radius_viz.py --ros-args -p odom_topic:=/odom

In RViz add a MarkerArray display on /turning_radius/markers.
The text marker (and the console, 1 Hz) shows:
  R      – fitted turning radius [m]
  delta  – effective steering angle atan(wheelbase / R) [rad]
and, if a steering command is seen on cmd_topic, the suggested
corrected steering_angle_to_servo_gain (gain * delta_cmd / delta_meas).
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def fit_circle(xs, ys):
    """Algebraic least-squares (Kasa) circle fit. Returns (cx, cy, r)."""
    A = np.column_stack([xs, ys, np.ones_like(xs)])
    b = xs ** 2 + ys ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(sol[2] + cx ** 2 + cy ** 2)
    return cx, cy, r


class TurningRadiusViz(Node):
    def __init__(self):
        super().__init__('turning_radius_viz')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/ackermann_cmd')
        self.declare_parameter('wheelbase', 0.335)
        self.declare_parameter('window_size', 400)     # points kept for the fit
        self.declare_parameter('min_spread', 0.3)      # [m] path extent before fitting
        self.declare_parameter('current_gain', -1.1)   # steering_angle_to_servo_gain in use

        self.wheelbase = self.get_parameter('wheelbase').value
        self.current_gain = self.get_parameter('current_gain').value
        self.min_spread = self.get_parameter('min_spread').value
        self.points = deque(maxlen=self.get_parameter('window_size').value)
        self.frame_id = 'odom'
        self.cmd_angle = None

        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped,
                                 self.get_parameter('cmd_topic').value,
                                 self.cmd_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/turning_radius/markers', 1)
        self.create_timer(0.5, self.fit_and_publish)
        self.get_logger().info(f'Listening on {odom_topic}; drive a steady circle.')

    def odom_cb(self, msg):
        self.frame_id = msg.header.frame_id or 'odom'
        p = msg.pose.pose.position
        self.points.append((p.x, p.y))

    def cmd_cb(self, msg):
        if abs(msg.drive.steering_angle) > 1e-3:
            self.cmd_angle = msg.drive.steering_angle

    def fit_and_publish(self):
        if len(self.points) < 20:
            return
        xs = np.array([p[0] for p in self.points])
        ys = np.array([p[1] for p in self.points])
        if max(xs.ptp(), ys.ptp()) < self.min_spread:
            return  # car not moving enough yet
        cx, cy, r = fit_circle(xs, ys)
        residual = float(np.std(np.hypot(xs - cx, ys - cy) - r))
        delta = math.atan(self.wheelbase / r)

        text = f'R={r:.3f}m  delta={delta:.4f}rad  fit_err={residual:.3f}m'
        if self.cmd_angle is not None:
            delta_meas = math.copysign(delta, self.cmd_angle)
            new_gain = self.current_gain * self.cmd_angle / delta_meas
            text += f'\ncmd={self.cmd_angle:.3f}rad  suggested_gain={new_gain:.3f}'
        self.get_logger().info(text.replace('\n', '  |  '), throttle_duration_sec=1.0)
        if self.marker_pub.get_subscription_count() > 0:  # rviz-off: skip marker
            self.marker_pub.publish(self.make_markers(cx, cy, r, text))

    def make_markers(self, cx, cy, r, text):
        now = self.get_clock().now().to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'turning_radius'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.a = 1.0
            return m

        circle = base(0, Marker.LINE_STRIP)
        circle.scale.x = 0.03
        circle.color.g = 1.0
        for th in np.linspace(0.0, 2.0 * math.pi, 100):
            pt = Point(x=cx + r * math.cos(th), y=cy + r * math.sin(th), z=0.05)
            circle.points.append(pt)

        path = base(1, Marker.LINE_STRIP)
        path.scale.x = 0.03
        path.color.r = 1.0
        path.color.g = 0.5
        for x, y in self.points:
            path.points.append(Point(x=x, y=y, z=0.07))

        center = base(2, Marker.SPHERE)
        center.scale.x = center.scale.y = center.scale.z = 0.15
        center.color.g = 1.0
        center.pose.position.x, center.pose.position.y = cx, cy

        label = base(3, Marker.TEXT_VIEW_FACING)
        label.scale.z = 0.3
        label.color.r = label.color.g = label.color.b = 1.0
        label.pose.position.x, label.pose.position.y = cx, cy
        label.pose.position.z = 0.5
        label.text = text

        arr = MarkerArray()
        arr.markers = [circle, path, center, label]
        return arr


def main():
    rclpy.init()
    node = TurningRadiusViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
