#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

WHEEL_RADIUS    = 0.05   
WHEEL_SEP       = 0.18  
ENC_DEADBAND    = 0.15 

_QOS_ODOM = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('wheel_radius',    WHEEL_RADIUS)
        self.declare_parameter('wheel_separation', WHEEL_SEP)
        self.declare_parameter('enc_deadband',    ENC_DEADBAND)
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_footprint')

        self._r   = float(self.get_parameter('wheel_radius').value)
        self._L   = float(self.get_parameter('wheel_separation').value)
        self._db  = float(self.get_parameter('enc_deadband').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._w_r = 0.0  
        self._w_l = 0.0  

        self._x   = 0.0
        self._y   = 0.0
        self._th  = 0.0
        self._t_prev = None

        self._tf = tf2_ros.TransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, '/odom', _QOS_ODOM)

        self.create_subscription(Float32, '/VelocityEncR', self._enc_r_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/VelocityEncL', self._enc_l_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._update)   # 20 Hz

        self.get_logger().info(
            f'odometry_node listo | r={self._r}m L={self._L}m deadband={self._db}rad/s')

    def _enc_r_cb(self, msg: Float32):
        self._w_r = msg.data

    def _enc_l_cb(self, msg: Float32):
        self._w_l = msg.data

    def _update(self):
        now = self.get_clock().now()
        if self._t_prev is None:
            self._t_prev = now
            return
        dt = (now - self._t_prev).nanoseconds * 1e-9
        self._t_prev = now
        if dt <= 0.0 or dt > 0.5:
            return

        w_r = self._w_r
        w_l = self._w_l

    
        if (abs(w_r) + abs(w_l)) * 0.5 < self._db:
            self._publish(now)
            return

        v_r = w_r * self._r
        v_l = w_l * self._r
        v   = (v_r + v_l) / 2.0
        w   = (v_r - v_l) / self._L

  
        th_mid = self._th + w * dt / 2.0
        self._x  += v * math.cos(th_mid) * dt
        self._y  += v * math.sin(th_mid) * dt
        self._th += w * dt
        self._th  = math.atan2(math.sin(self._th), math.cos(self._th))

        self._publish(now)

    def _publish(self, now):
        stamp = now.to_msg()
        th = self._th

       
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id  = self._base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(th / 2.0)
        tf.transform.rotation.w = math.cos(th / 2.0)
        self._tf.sendTransform(tf)

      
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)
        w_r = self._w_r * self._r
        w_l = self._w_l * self._r
        msg.twist.twist.linear.x  = (w_r + w_l) / 2.0
        msg.twist.twist.angular.z = (w_r - w_l) / self._L
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.02
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
