#!/usr/bin/env python3


import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _blend_angle(a, b, alpha):
    return _norm_angle(a + alpha * _norm_angle(b - a))


class ArucoMapOdom(Node):
    def __init__(self):
        super().__init__('aruco_map_odom')

        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('aruco_pose_topic',  '/aruco/pose')
        self.declare_parameter('map_to_odom_topic', '/map_to_odom')
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('publish_rate_hz',   20.0)
        self.declare_parameter('correction_alpha',  0.35)
   
        self.declare_parameter('yaw_alpha',         0.07)
        self.declare_parameter('max_correction_step_m',   0.35)
        self.declare_parameter('max_correction_step_yaw', 0.70)
        self.declare_parameter('max_odom_age',      0.30)
        self.declare_parameter('map_min_x', 0.0)
        self.declare_parameter('map_max_x', 3.76)
        self.declare_parameter('map_min_y', 0.0)
        self.declare_parameter('map_max_y', 4.86)
        self.declare_parameter('map_bounds_margin', 0.25)
        self.declare_parameter('correct_yaw',       False)

        self._map_frame  = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._alpha      = float(self.get_parameter('correction_alpha').value)
        self._yaw_alpha  = float(self.get_parameter('yaw_alpha').value)
        self._max_step_m   = float(self.get_parameter('max_correction_step_m').value)
        self._max_step_yaw = float(self.get_parameter('max_correction_step_yaw').value)
        self._max_odom_age = float(self.get_parameter('max_odom_age').value)
        self._map_min_x  = float(self.get_parameter('map_min_x').value)
        self._map_max_x  = float(self.get_parameter('map_max_x').value)
        self._map_min_y  = float(self.get_parameter('map_min_y').value)
        self._map_max_y  = float(self.get_parameter('map_max_y').value)
        self._map_margin   = float(self.get_parameter('map_bounds_margin').value)
        self._correct_yaw  = bool(self.get_parameter('correct_yaw').value)

        self._odom_pose  = None
        self._odom_stamp = None
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._initialized = False

        self._tf  = tf2_ros.TransformBroadcaster(self)
        self._pub = self.create_publisher(
            TransformStamped,
            self.get_parameter('map_to_odom_topic').value, 10)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('aruco_pose_topic').value, self._aruco_cb, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('aruco_map_odom listo')

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._odom_pose  = (msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            _yaw_from_quat(q))
        self._odom_stamp = self.get_clock().now()

    def _aruco_cb(self, msg: PoseWithCovarianceStamped):
        if self._odom_pose is None or self._odom_stamp is None:
            self.get_logger().warn('Sin /odom todavía', throttle_duration_sec=2.0)
            return

        odom_age = (self.get_clock().now() - self._odom_stamp).nanoseconds * 1e-9
        if odom_age > self._max_odom_age:
            self.get_logger().warn(f'odom muy viejo {odom_age:.2f}s', throttle_duration_sec=2.0)
            return

        q = msg.pose.pose.orientation
        map_base = (msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    _yaw_from_quat(q))

        if not self._inside_map(map_base):
            self.get_logger().warn(
                f'Pose ArUco fuera del mapa: ({map_base[0]:.2f},{map_base[1]:.2f})',
                throttle_duration_sec=1.0)
            return

        odom_base = self._odom_pose

  
        aruco_yaw = _norm_angle(map_base[2] - odom_base[2])

  
        yaw_accepted = False
        if not self._initialized:
     
            yaw = aruco_yaw
            yaw_accepted = True
        elif self._correct_yaw:
            yaw_step = abs(_norm_angle(aruco_yaw - self._yaw))
            if yaw_step > self._max_step_yaw:
                self.get_logger().warn(
                    f'Yaw rechazado: {math.degrees(yaw_step):.1f}deg '
                    f'> {math.degrees(self._max_step_yaw):.1f}deg',
                    throttle_duration_sec=2.0)
                yaw = self._yaw  # solo corregir X,Y con yaw actual
            else:
                yaw = aruco_yaw
                yaw_accepted = True
        else:
            yaw = self._yaw

        c = math.cos(yaw)
        s = math.sin(yaw)
        x = map_base[0] - (odom_base[0] * c - odom_base[1] * s)
        y = map_base[1] - (odom_base[0] * s + odom_base[1] * c)

        if not self._initialized:
            self._x   = x
            self._y   = y
            self._yaw = yaw
            self._initialized = True
            self.get_logger().info(
                f'map->odom inicial: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg'
                f' [correct_yaw={self._correct_yaw}]')
            return

        step = math.hypot(x - self._x, y - self._y)

 
        _MAX_MEDIUM = self._max_step_m * 3.0  
        _MAX_LARGE  = 7.0                      
        if step > _MAX_LARGE:
            self.get_logger().warn(
                f'Correccion rechazada (imposible): {step:.2f}m > {_MAX_LARGE:.2f}m',
                throttle_duration_sec=2.0)
            return
        if step > _MAX_MEDIUM:
            a_pos = 0.04   
            self.get_logger().warn(
                f'Correccion GRANDE aceptada: {step:.2f}m > {_MAX_MEDIUM:.2f}m '
                f'(alpha={a_pos:.2f}, drift multi-vuelta)',
                throttle_duration_sec=3.0)
        elif step > self._max_step_m:
            a_pos = 0.08  
        else:
            a_pos = min(0.45, max(0.20, self._alpha))

        self._x += a_pos * (x - self._x)
        self._y += a_pos * (y - self._y)

        if self._correct_yaw and yaw_accepted:
            self._yaw = _blend_angle(self._yaw, aruco_yaw, self._yaw_alpha)

        self.get_logger().info(
            f'map->odom: x={self._x:.3f} y={self._y:.3f} yaw={math.degrees(self._yaw):.1f}deg '
            f'[err={step:.2f}m a_pos={a_pos:.2f} a_yaw={self._yaw_alpha:.2f}]',
            throttle_duration_sec=1.0)

    def _inside_map(self, pose):
        x, y, _ = pose
        m = max(0.0, self._map_margin)
        return (self._map_min_x - m <= x <= self._map_max_x + m and
                self._map_min_y - m <= y <= self._map_max_y + m)

    def _publish(self):
        tf = TransformStamped()
        tf.header.stamp    = self.get_clock().now().to_msg()
        tf.header.frame_id = self._map_frame
        tf.child_frame_id  = self._odom_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = math.sin(self._yaw / 2.0)
        tf.transform.rotation.w = math.cos(self._yaw / 2.0)
        self._tf.sendTransform(tf)
        self._pub.publish(tf)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMapOdom()
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
