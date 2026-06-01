#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')

        self.declare_parameter('input_topic',      '/scan')
        self.declare_parameter('target_frame',     'lidar_link')
        self.declare_parameter('invert_angles',    False)
        self.declare_parameter('angle_offset_rad', math.pi)

        in_topic     = self.get_parameter('input_topic').value
        self.frame   = self.get_parameter('target_frame').value
        self.invert  = bool(self.get_parameter('invert_angles').value)
        self.offset  = float(self.get_parameter('angle_offset_rad').value)

        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, '/scan_stamped', out_qos)
        self.sub = self.create_subscription(
            LaserScan, in_topic, self._cb, qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'scan_restamper: {in_topic} -> /scan_stamped '
            f'frame={self.frame} invert={self.invert} offset={self.offset:.4f}rad')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'invert_angles':
                self.invert = bool(p.value)
                self.get_logger().info(f'invert_angles -> {self.invert}')
            elif p.name == 'angle_offset_rad':
                self.offset = float(p.value)
                self.get_logger().info(f'angle_offset_rad -> {self.offset:.4f}')
        return SetParametersResult(successful=True)

    def _cb(self, msg: LaserScan):
       
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        if self.invert:
            msg.ranges = list(reversed(msg.ranges))
            if msg.intensities:
                msg.intensities = list(reversed(msg.intensities))
            old_min = msg.angle_min
            msg.angle_min = -msg.angle_max
            msg.angle_max = -old_min

        if self.offset != 0.0:
            msg.angle_min += self.offset
            msg.angle_max += self.offset

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
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
