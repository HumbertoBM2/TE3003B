"""
Nodo de navegación autónoma para Puzzlebot (montacargas).
Implementado desde cero sin librerías de navegación prefabricadas.

Funcionalidad:
  - Recibe lista de waypoints objetivo
  - Usa la odometría del EKF-SLAM para saber dónde está el robot
  - Evita obstáculos con datos del LiDAR (campo potencial simple)
  - Controla los motores publicando velocidades a ControlR / ControlL
  - Se detiene cuando llega al waypoint con tolerancia DMIN

Algoritmo de evasión de obstáculos:
  - Fuerza atractiva hacia el waypoint objetivo (proporcional a distancia)
  - Fuerza repulsiva de obstáculos cercanos detectados por LiDAR
  - Suma vectorial → velocidad lineal y angular del robot

Tópicos suscritos:
  /slam/odom           (nav_msgs/Odometry)      — pose del EKF-SLAM
  /scan                (sensor_msgs/LaserScan)   — datos LiDAR
  /nav/goal            (geometry_msgs/PoseStamped) — waypoint objetivo

Tópicos publicados:
  /cmd_vel             (geometry_msgs/Twist) — velocidad lineal y angular al firmware ESP32
  /nav/status          (std_msgs/String)     — estado actual del navegador
"""

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
import numpy as np
from typing import List, Optional, Tuple

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist

from .my_math import wrap_to_pi, euler_from_quaternion


# ─────────────────────────────────────────────────────────────────────────────
# Parámetros del robot
# ─────────────────────────────────────────────────────────────────────────────
WHEEL_RADIUS   = 0.05       # metros
ROBOT_WIDTH    = 0.1875     # metros

# Límites de velocidad
V_MAX          = 0.20       # m/s
W_MAX          = 1.20       # rad/s

# Control goto
Kd_att         = 1.0        # ganancia atractiva (distancia)
Kt_att         = 2.5        # ganancia atractiva (ángulo)
DMIN_ARRIVE    = 0.10       # metros — radio de llegada

# Campo potencial repulsivo (LiDAR)
D_OBS_LIMIT    = 0.50       # metros — distancia mínima a obstáculo antes de repeler
K_REP          = 0.8        # ganancia fuerza repulsiva

# PID motores — ya no se usan (el firmware ESP32 los maneja internamente)
# El navigator solo publica /cmd_vel (v, w) y el ESP32 convierte a PWM
Kp_motor       = 0.03
Ti_motor       = 0.05
Td_motor       = 0.0


class Navigator(Node):
    """Nodo ROS 2 de navegación autónoma con evasión de obstáculos."""

    def __init__(self):
        super().__init__('navigator')

        qos_s = qos.qos_profile_sensor_data

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist,  '/cmd_vel',    qos_cmd)
        self.pub_status  = self.create_publisher(String, '/nav/status', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,     '/slam/odom', self._odom_cb,   qos_s)
        self.create_subscription(LaserScan,    '/scan',      self._lidar_cb,  qos_s)
        self.create_subscription(PoseStamped,  '/nav/goal',  self._goal_cb,   10)

        # ── Estado del robot ─────────────────────────────────────────────────
        self.pose_x     = 0.0
        self.pose_y     = 0.0
        self.pose_theta = 0.0
        self.odom_ready = False

        # ── LiDAR ────────────────────────────────────────────────────────────
        self.laser_ranges:  List[float] = []
        self.laser_angle_min  = 0.0
        self.laser_angle_incr = 0.0

        # ── Waypoint objetivo ─────────────────────────────────────────────────
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.navigating = False
        self.stop_sent = True
        self.last_wait_log_ns = 0

        # ── Timers ────────────────────────────────────────────────────────────
        self.dt_nav = 0.10   # 10 Hz lógica de navegación
        self.create_timer(self.dt_nav, self._nav_loop)

        self.get_logger().info(
            'Navegador iniciado. Esperando /slam/odom y goal en /nav/goal...'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        _, _, self.pose_theta = euler_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

    def _lidar_cb(self, msg: LaserScan):
        self.laser_ranges     = list(msg.ranges)
        self.laser_angle_min  = msg.angle_min
        self.laser_angle_incr = msg.angle_increment

    def _goal_cb(self, msg: PoseStamped):
        self.goal_x    = msg.pose.position.x
        self.goal_y    = msg.pose.position.y
        self.navigating = True
        self.stop_sent = False
        self.get_logger().info(
            f'Nuevo goal recibido: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Loop de navegación (10 Hz) — calcula setpoints de ruedas
    # ─────────────────────────────────────────────────────────────────────────

    def _nav_loop(self):
        if not self.odom_ready:
            if self.navigating:
                self._send_stop_once()
                self._log_waiting('Esperando /slam/odom; no mando avance todavia.')
            return

        if not self.navigating:
            self._log_waiting('Sin goal activo en /nav/goal; no publico /cmd_vel.')
            return

        dx    = self.goal_x - self.pose_x
        dy    = self.goal_y - self.pose_y
        dist  = math.sqrt(dx**2 + dy**2)

        # ── Llegamos al waypoint ──────────────────────────────────────────
        if dist < DMIN_ARRIVE:
            self._send_stop_once()
            self.navigating = False
            self._publish_status('ARRIVED')
            self.get_logger().info(
                f'Waypoint alcanzado: ({self.goal_x:.2f}, {self.goal_y:.2f})'
            )
            return

        # ── Velocidad de campo potencial ──────────────────────────────────
        # Fuerza atractiva: vc proporcional a distancia, wc a error de ángulo
        target_angle = math.atan2(dy, dx)
        angle_error  = wrap_to_pi(target_angle - self.pose_theta)

        vc = Kd_att * dist
        wc = Kt_att * angle_error

        # ── Fuerza repulsiva del LiDAR ────────────────────────────────────
        if self.laser_ranges:
            rep_x, rep_y = self._compute_repulsive_force()
            # Proyectar fuerza repulsiva al marco del robot
            rep_forward = rep_x * math.cos(self.pose_theta) + rep_y * math.sin(self.pose_theta)
            rep_lateral = -rep_x * math.sin(self.pose_theta) + rep_y * math.cos(self.pose_theta)

            vc += rep_forward
            wc += Kt_att * math.atan2(rep_lateral, max(abs(rep_forward), 0.01))

        # ── Saturar velocidades ───────────────────────────────────────────
        vc = float(np.clip(vc, -V_MAX, V_MAX))
        wc = float(np.clip(wc, -W_MAX, W_MAX))

        self._send_cmd_vel(vc, wc)
        self._publish_status(f'NAVIGATING dist={dist:.2f}')

    # ─────────────────────────────────────────────────────────────────────────
    # Campo potencial repulsivo desde LiDAR
    # ─────────────────────────────────────────────────────────────────────────

    def _compute_repulsive_force(self) -> Tuple[float, float]:
        """Retorna vector (fx, fy) en frame global de la fuerza repulsiva."""
        fx, fy = 0.0, 0.0

        for i, r in enumerate(self.laser_ranges):
            if not math.isfinite(r) or r <= 0.0 or r > D_OBS_LIMIT:
                continue

            angle_global = (self.laser_angle_min + i * self.laser_angle_incr
                            + self.pose_theta)

            # Fuerza inversamente proporcional al cuadrado de la distancia
            magnitude = K_REP * (1.0 / r - 1.0 / D_OBS_LIMIT) / (r**2)

            # Dirección: se aleja del obstáculo
            fx -= magnitude * math.cos(angle_global)
            fy -= magnitude * math.sin(angle_global)

        # Saturar para estabilidad
        norm = math.sqrt(fx**2 + fy**2)
        if norm > V_MAX:
            fx = fx / norm * V_MAX
            fy = fy / norm * V_MAX

        return fx, fy

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _send_cmd_vel(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd_vel.publish(msg)
        self.stop_sent = abs(v) < 1e-6 and abs(w) < 1e-6

    def _send_stop_once(self):
        if not self.stop_sent:
            self._send_cmd_vel(0.0, 0.0)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

    def _log_waiting(self, message: str):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_wait_log_ns > 5_000_000_000:
            self.get_logger().info(message)
            self.last_wait_log_ns = now_ns


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
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
