#!/usr/bin/env python3
"""
Lane visualization for rviz, independent of the overtake planner.

Rebuilds the left/middle/right lanes (track centerline +/- lane_offset) from
/centerline_waypoints and publishes them on /lane_markers so the lanes are visible
in every mode. Viz only -- it publishes nothing a planner or the controller consumes.

Geometry: same as change_avoidance_node.generate_lanes (centerline +/- lane_offset
along the normal), but the normal is taken from the travel direction of successive
centerline points (left = +90 deg of the tangent), so no psi convention / map
orientation swap is needed. Offsets are clamped inside the track bounds.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from f110_msgs.msg import WpntArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def build_lanes(xy: np.ndarray, d_left: np.ndarray, d_right: np.ndarray,
                offset: float, margin: float = 0.1):
    """
    xy: (N,2) closed-loop centerline (last point may duplicate the first).
    d_left/d_right: half widths at each point (centerline wpnt fields).
    Returns dict name -> (M,2) polyline: 'left', 'middle', 'right'.
    """
    # drop a duplicated closing point so the wrap-around tangent is not zero
    if len(xy) > 1 and np.allclose(xy[0], xy[-1]):
        xy, d_left, d_right = xy[:-1], d_left[:-1], d_right[:-1]
    nxt = np.roll(xy, -1, axis=0)
    prv = np.roll(xy, 1, axis=0)
    tangent = nxt - prv
    norm = np.hypot(tangent[:, 0], tangent[:, 1])
    norm[norm < 1e-9] = 1e-9
    tangent /= norm[:, None]
    left_normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
    off_l = np.minimum(offset, np.maximum(d_left - margin, 0.0))
    off_r = np.minimum(offset, np.maximum(d_right - margin, 0.0))
    return {
        'left': xy + left_normal * off_l[:, None],
        'middle': xy.copy(),
        'right': xy - left_normal * off_r[:, None],
    }


class LaneViz(Node):
    COLOURS = {
        'left': (0.1, 0.9, 0.1),    # green, like change_avoidance_node outer_left
        'middle': (1.0, 1.0, 1.0),  # white, like its center
        'right': (0.9, 0.1, 0.1),   # red, like its inner_right
    }

    def __init__(self):
        super().__init__('lane_viz')
        self.declare_parameter('lane_offset', 0.3)
        self.declare_parameter('rate_hz', 1.0)
        self.centerline = None
        self.sub = self.create_subscription(WpntArray, '/centerline_waypoints', self.centerline_cb, 10)
        self.pub = self.create_publisher(MarkerArray, '/lane_markers', 10)
        period = 1.0 / max(float(self.get_parameter('rate_hz').value), 0.1)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info("lane_viz: waiting for /centerline_waypoints")

    def centerline_cb(self, msg: WpntArray):
        self.centerline = msg

    def timer_cb(self):
        if self.centerline is None or not self.centerline.wpnts:
            return
        if self.pub.get_subscription_count() == 0:
            return
        wp = self.centerline.wpnts
        xy = np.array([[w.x_m, w.y_m] for w in wp], dtype=float)
        d_left = np.array([w.d_left for w in wp], dtype=float)
        d_right = np.array([w.d_right for w in wp], dtype=float)
        offset = float(self.get_parameter('lane_offset').value)
        lanes = build_lanes(xy, d_left, d_right, offset)

        stamp = self.get_clock().now().to_msg()
        mrks = MarkerArray()
        for mid, name in enumerate(('left', 'middle', 'right')):
            pts = lanes[name]
            mrk = Marker(header=Header(stamp=stamp, frame_id='map'))
            mrk.ns = f'lane_{name}'
            mrk.id = mid
            mrk.type = Marker.LINE_STRIP
            mrk.action = Marker.ADD
            mrk.scale.x = 0.03
            mrk.color.r, mrk.color.g, mrk.color.b = self.COLOURS[name]
            mrk.color.a = 1.0
            mrk.pose.orientation.w = 1.0
            mrk.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in pts]
            mrk.points.append(Point(x=float(pts[0][0]), y=float(pts[0][1]), z=0.0))  # close loop
            mrks.markers.append(mrk)
        self.pub.publish(mrks)


def main(args=None):
    rclpy.init(args=args)
    node = LaneViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
