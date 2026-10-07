#!/usr/bin/env python3
"""Intensity-based pointcloud filter for the obstacle-detection path.

Subscribes the GL-5 driver cloud (x,y,z,intensity) and republishes it with weak
mid-range returns removed:

    drop point  iff  r_min < range < r_max  AND  intensity < intensity_thresh

Weak returns in that band are grazing-angle artifacts off smooth/glossy walls;
they flicker scan-to-scan and detect clusters them into phantom obstacles. Near returns are always strong, and beyond r_max nothing is
dropped so long-range detection is never capped.

The filtered cloud feeds a second pointcloud_to_laserscan -> /scan_obs used by
detect/tracking ONLY. Cartographer keeps the unfiltered /scan: deleting far
wall returns would starve localization of constraints.

All three parameters are dynamic (ros2 param set /scan_intensity_filter ...).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class IntensityFilter(Node):
    def __init__(self):
        super().__init__('scan_intensity_filter')
        self.declare_parameter('intensity_thresh', 12.0)
        self.declare_parameter('r_min', 3.0)
        self.declare_parameter('r_max', 7.0)

        self.pub = self.create_publisher(PointCloud2, '/soslab/pointcloud_obs', 5)
        self.sub = self.create_subscription(
            PointCloud2, '/soslab/pointcloud', self.cb, qos_profile_sensor_data)
        self.warned_layout = False

    def cb(self, msg: PointCloud2):
        thresh = self.get_parameter('intensity_thresh').value
        r_min = self.get_parameter('r_min').value
        r_max = self.get_parameter('r_max').value

        # Fast path expects the SDK driver layout: 4 packed FLOAT32 (x,y,z,intensity).
        if msg.point_step != 16 or len(msg.fields) != 4 or msg.fields[3].name != 'intensity':
            if not self.warned_layout:
                self.get_logger().warn(
                    'Unexpected cloud layout (not x,y,z,intensity float32) — passing through unfiltered.')
                self.warned_layout = True
            self.pub.publish(msg)
            return

        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        r = np.linalg.norm(pts[:, :2], axis=1)
        drop = (r > r_min) & (r < r_max) & (pts[:, 3] < thresh)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = msg.is_dense
        kept = pts[~drop]
        out.width = kept.shape[0]
        out.row_step = out.point_step * out.width
        out.data = kept.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IntensityFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
