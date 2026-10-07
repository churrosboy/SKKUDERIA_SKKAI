#!/usr/bin/env python3
"""Gate the odometry fed to Cartographer during a collision.

After a crash the wheels keep spinning against the (flexible) wall or in a
gap: the VESC reports that erpm as real speed, vesc_to_odom -> EKF ->
/early_fusion/odom integrates it, and cartographer_node (use_odometry =
true, ceres weights x0.2) follows the bogus motion prior -> the map pose
"flies away". While the user pushes the car back (motor off) the same
odometry reads ~0 instead, which is just as wrong.

This node relays /early_fusion/odom to /early_fusion/odom_gated and, while
collision_stop reports /collision_detected, HOLDS it: the last pose is
republished frozen (twist zeroed, current stamp) so cartographer sees a
"car is stationary" motion prior at full rate. NOT dropped: cartographer's
collator blocks on the emptiest sensor queue ("Queue waiting for data:
(N, odom)"), so a silent odom topic would freeze scan processing, TF and
/car_state/odom for the whole recovery (and break collision_stop's escape
check). Nothing changes during normal driving: every frame is forwarded.
Extra latency of the python hop: measure with
`ros2 topic delay /early_fusion/odom_gated` (port to C++ if > ~10 ms).

Kill switch: gate_enable:=False (launch arg odom_gate) = plain relay.
"""
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class OdomGate(Node):

    def __init__(self):
        super().__init__('odom_gate')
        self.declare_parameter('gate_enable', True)
        self.declare_parameter('collision_max_age', 0.5)   # s
        self.declare_parameter('input_topic', '/early_fusion/odom')
        self.declare_parameter('output_topic', '/early_fusion/odom_gated')
        gp = self.get_parameter
        self.gate_enable = gp('gate_enable').value
        self.collision_max_age = gp('collision_max_age').value
        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value

        self.collision = False
        self.collision_time = None
        self.gated = False          # for the open/close log lines
        self.dropped = 0
        self.hold = None            # frozen Odometry while gated

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self.odom_cb, 10)
        self.create_subscription(
            Bool, '/collision_detected', self.collision_cb, 1)
        self.get_logger().info(
            f'odom gate {in_topic} -> {out_topic} '
            f'({"ARMED" if self.gate_enable else "relay only"})')

    def collision_cb(self, msg: Bool):
        self.collision = msg.data
        self.collision_time = self.get_clock().now().nanoseconds * 1e-9

    def blocked(self):
        if not self.gate_enable or not self.collision or \
                self.collision_time is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        # staleness guard: if collision_stop dies mid-recovery, reopen
        return (now - self.collision_time) < self.collision_max_age

    def odom_cb(self, msg: Odometry):
        if self.blocked():
            if not self.gated:
                self.gated = True
                self.dropped = 0
                self.hold = msg      # freeze the pose at the trip
                self.get_logger().warn(
                    'collision: odometry to cartographer HELD (frozen pose)')
            self.dropped += 1
            out = Odometry()
            out.header = msg.header          # current stamp keeps the collator fed
            out.child_frame_id = msg.child_frame_id
            out.pose = self.hold.pose
            out.twist.covariance = msg.twist.covariance   # twist itself stays zero
            self.pub.publish(out)
            return
        if self.gated:
            self.gated = False
            self.hold = None
            self.get_logger().warn(
                f'odometry to cartographer resumed ({self.dropped} frames held)')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
