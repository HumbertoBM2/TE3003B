#!/usr/bin/env python3


import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

MAX_LINEAR  = 0.25
MAX_ANGULAR = 1.5
CMD_TIMEOUT = 0.30  


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.declare_parameter('max_linear_vel',  MAX_LINEAR)
        self.declare_parameter('max_angular_vel', MAX_ANGULAR)

        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)

        self._cmd_v    = 0.0
        self._cmd_w    = 0.0
        self._cmd_time = 0.0

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_out)
        self.create_subscription(Twist, '/cmd_vel_keyboard', self._cmd_cb,
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                            history=HistoryPolicy.KEEP_LAST, depth=10))
        self.create_timer(0.05, self._timer)   # 20 Hz

        self.get_logger().info('cmd_vel relay activo')

    def _cmd_cb(self, msg: Twist):
        self._cmd_v    = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        self._cmd_w    = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self._cmd_time = time.monotonic()

    def _timer(self):
        if self._cmd_time == 0.0:
            return
        if time.monotonic() - self._cmd_time > CMD_TIMEOUT:
            self._cmd_v = 0.0
            self._cmd_w = 0.0
        out = Twist()
        out.linear.x  = self._cmd_v
        out.angular.z = self._cmd_w
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
