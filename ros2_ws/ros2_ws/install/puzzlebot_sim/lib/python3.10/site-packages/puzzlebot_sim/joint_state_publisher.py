"""
joint_state_publisher.py
========================
ROS 2 node that drives the Puzzlebot simulation in RVIZ.

Responsibilities
----------------
1. Broadcast the dynamic transform  odom -> base_footprint.
   This represents the robot's pose in the odometry frame and is updated at
   every timer tick so the robot moves around a circular path in RVIZ.

2. Publish /joint_states for the two continuous wheel joints
   (wheel_r_joint, wheel_l_joint).  robot_state_publisher reads these
   messages and rebroadcasts the corresponding TF transforms.

The remaining transforms in the TF tree are handled automatically by
robot_state_publisher from the URDF:
  - base_footprint  -> base_link    (fixed joint)
  - base_link       -> wheel_r_link (continuous joint, driven by /joint_states)
  - base_link       -> wheel_l_link (continuous joint, driven by /joint_states)
  - base_link       -> caster_link  (fixed joint)

Motion model
------------
The robot follows a circular orbit for visualisation purposes:
  - Orbit radius   : ORBIT_RADIUS  metres
  - Orbit speed    : ORBIT_OMEGA   rad/s
  - Wheel radius   : WHEEL_RADIUS  metres  (used to compute wheel spin rate)

At every timer tick the pose of base_footprint in the odom frame is:
  x(t) = ORBIT_RADIUS * cos(ORBIT_OMEGA * t)
  y(t) = ORBIT_RADIUS * sin(ORBIT_OMEGA * t)
  yaw  = ORBIT_OMEGA * t  (robot always faces its direction of travel)

The linear velocity of the robot centre is  v = ORBIT_RADIUS * ORBIT_OMEGA.
Both wheels spin at the same rate  w_wheel = v / WHEEL_RADIUS  because the
orbit is a pure circle (no differential steering required for straight arcs).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------------------
# Simulation constants
# ---------------------------------------------------------------------------
TIMER_PERIOD = 0.02       # seconds  (50 Hz update rate)
ORBIT_RADIUS = 0.5        # metres   – radius of the circular demo path
ORBIT_OMEGA  = 0.4        # rad/s    – angular speed along the orbit left wheel
ORBIT_OMEGA2  = 0.2        # rad/s    – angular speed along the orbit right wheel

WHEEL_RADIUS = 0.05       # metres   – Puzzlebot wheel radius


class PuzzlebotPublisher(Node):
    """Publishes odom->base_footprint TF and wheel joint states for the Puzzlebot."""

    def __init__(self):
        super().__init__('puzzlebot_publisher')

        # Dynamic TF broadcaster (odom -> base_footprint)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Pre-allocate the TransformStamped message
        self._base_footprint_tf = self._build_transform('odom', 'base_footprint')

        # Pre-allocate the JointState message
        self._joint_state = JointState()
        self._joint_state.name = ['wheel_r_joint', 'wheel_l_joint']
        self._joint_state.position = [0.0, 0.0]
        self._joint_state.velocity = [0.0, 0.0]
        self._joint_state.effort   = [0.0, 0.0]

        # Publisher for /joint_states (consumed by robot_state_publisher)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Record the node start time so that t=0 coincides with node launch.
        # Using epoch time directly causes the orbit to start at an arbitrary
        # phase and the per-tick position delta becomes numerically negligible
        # relative to the large absolute timestamp value.
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        # Periodic callback
        self.create_timer(TIMER_PERIOD, self._timer_cb)

        self.get_logger().info('PuzzlebotPublisher started. Robot will orbit the odom origin.')

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _timer_cb(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0  # seconds since node start

        # --- Pose of base_footprint in the odom frame ---
        x   = ORBIT_RADIUS * math.cos(ORBIT_OMEGA * t)
        y   = ORBIT_RADIUS * math.sin(ORBIT_OMEGA * t)
        # The tangent direction of a circle is 90 deg ahead of the radial direction.
        # Without the pi/2 offset the robot's +X axis points radially (toward/away
        # from the centre) instead of along the direction of travel.
        yaw = ORBIT_OMEGA * t + math.pi / 2.0

        stamp = self.get_clock().now().to_msg()

        self._base_footprint_tf.header.stamp = stamp
        self._base_footprint_tf.transform.translation.x = x
        self._base_footprint_tf.transform.translation.y = y
        self._base_footprint_tf.transform.translation.z = 0.0

        # Yaw-only quaternion around Z: (w, x, y, z) = (cos(a/2), 0, 0, sin(a/2))
        # No external library needed.
        self._base_footprint_tf.transform.rotation.w = math.cos(yaw / 2.0)
        self._base_footprint_tf.transform.rotation.x = 0.0
        self._base_footprint_tf.transform.rotation.y = 0.0
        self._base_footprint_tf.transform.rotation.z = math.sin(yaw / 2.0)

        self._tf_broadcaster.sendTransform(self._base_footprint_tf)

        # --- Wheel joint positions ---
        # Linear velocity of the robot: v = ORBIT_RADIUS * ORBIT_OMEGA
        # Wheel angular position:       theta_wheel = (v / WHEEL_RADIUS) * t
        wheel_angle_r = (ORBIT_RADIUS * ORBIT_OMEGA / WHEEL_RADIUS) * t
        wheel_angle_l = (ORBIT_RADIUS * ORBIT_OMEGA2 / WHEEL_RADIUS) * t

        self._joint_state.header.stamp = stamp
        self._joint_state.position[0] = wheel_angle_r   # wheel_r_joint
        self._joint_state.position[1] = wheel_angle_l   # wheel_l_joint

        self._joint_pub.publish(self._joint_state)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _build_transform(parent: str, child: str) -> TransformStamped:
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id  = child
        tf.transform.rotation.w = 1.0  # identity quaternion
        return tf


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
