#!/usr/bin/env python3
"""
Relay + corrección de desbalance: /cmd_vel_keyboard → /cmd_vel (BEST_EFFORT)

Arquitectura:
  1. _cmd_cb     — aplica left_scale y guarda el comando base
  2. _pi_timer   — 20 Hz: lee encoders, aplica corrección PI, publica cmd_vel
                   El timer es el ÚNICO publicador → salida consistente a 20 Hz.
  3. param_cb    — actualiza left_scale / kp / ki en caliente (ros2 param set)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

MAX_LINEAR  = 0.25
MAX_ANGULAR = 1.5

WHEEL_RADIUS = 0.05
WHEEL_BASE   = 0.18     # metros — 18 cm medidos

LEFT_SCALE  = 0.52      # < 1.0 → frena la izquierda (2x más rápida que la derecha)
KP          = 1.5
KI          = 0.30
INT_MAX     = 3.0
ENC_ALPHA   = 0.3
ENC_MIN     = 0.15      # rad/s — umbral mínimo en encoders para activar PI
CMD_TIMEOUT = 0.30      # s — si no llega cmd_vel en este tiempo, publicar 0


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.declare_parameter('max_linear_vel', MAX_LINEAR)
        self.declare_parameter('max_angular_vel', MAX_ANGULAR)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        self.declare_parameter('left_scale', LEFT_SCALE)
        self.declare_parameter('kp', KP)
        self.declare_parameter('ki', KI)
        self.declare_parameter('int_max', INT_MAX)
        self.declare_parameter('enc_alpha', ENC_ALPHA)

        self.max_lin    = float(self.get_parameter('max_linear_vel').value)
        self.max_ang    = float(self.get_parameter('max_angular_vel').value)
        self.wheel_r    = float(self.get_parameter('wheel_radius').value)
        self.wheel_L    = float(self.get_parameter('wheel_base').value)
        self.left_scale = float(self.get_parameter('left_scale').value)
        self.kp         = float(self.get_parameter('kp').value)
        self.ki         = float(self.get_parameter('ki').value)
        self.int_max    = float(self.get_parameter('int_max').value)
        self.alpha      = float(self.get_parameter('enc_alpha').value)

        # Comando base (después de left_scale, antes de PI)
        self._cmd_v    = 0.0
        self._cmd_w    = 0.0
        self._cmd_time = 0.0   # monotonic timestamp del último cmd_vel

        # Estado PI
        self._v_r = 0.0
        self._v_l = 0.0
        self._int = 0.0
        self._enc_received = False
        self._diag_correction = 0.0
        self._pi_activations  = 0

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_out)
        self.create_subscription(Twist,   '/cmd_vel_keyboard', self._cmd_cb, qos_in)
        self.create_subscription(Float32, 'VelocityEncR', self._enc_r_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL', self._enc_l_cb, qos_profile_sensor_data)

        # Timer de corrección PI — único publicador de /cmd_vel
        self.create_timer(0.05, self._pi_timer)   # 20 Hz
        self.create_timer(3.0,  self._diag)

        # Callbacks para cambios de parámetros en caliente
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'cmd_vel relay activo | left_scale={self.left_scale:.3f} '
            f'Kp={self.kp} Ki={self.ki} | '
            f'wheel_r={self.wheel_r}m L={self.wheel_L}m'
        )

    # ── Callback parámetros dinámicos ─────────────────────────────────────────

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'left_scale':
                self.left_scale = float(p.value)
                self.get_logger().info(f'[relay] left_scale → {self.left_scale:.3f}')
            elif p.name == 'kp':
                self.kp = float(p.value)
                self.get_logger().info(f'[relay] Kp → {self.kp}')
            elif p.name == 'ki':
                self.ki = float(p.value)
                self.get_logger().info(f'[relay] Ki → {self.ki}')
        return SetParametersResult(successful=True)

    # ── Encoders ──────────────────────────────────────────────────────────────

    def _enc_r_cb(self, msg: Float32):
        self._v_r = self.alpha * self._v_r + (1.0 - self.alpha) * msg.data
        self._enc_received = True

    def _enc_l_cb(self, msg: Float32):
        self._v_l = self.alpha * self._v_l + (1.0 - self.alpha) * msg.data
        self._enc_received = True

    # ── Recibir comando de teleop — solo guarda, no publica ───────────────────

    def _cmd_cb(self, msg: Twist):
        v     = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        w_cmd = msg.angular.z

        # left_scale feedforward: reduce setpoint de la rueda izquierda
        L   = self.wheel_L
        v_r = v + w_cmd * L / 2.0
        v_l = (v - w_cmd * L / 2.0) * self.left_scale
        v   = (v_r + v_l) / 2.0
        w   = (v_r - v_l) / L

        self._cmd_v    = v
        self._cmd_w    = w
        self._cmd_time = time.monotonic()

    # ── Timer PI — único publicador ───────────────────────────────────────────

    def _pi_timer(self):
        # No publicar NADA hasta recibir el primer cmd_vel_keyboard.
        # Sin esto, el timer pelea con teleop si se corre sin el remap.
        if self._cmd_time == 0.0:
            return

        now = time.monotonic()
        v = self._cmd_v
        w = self._cmd_w

        # Si el comando es demasiado viejo → parar el robot
        if now - self._cmd_time > CMD_TIMEOUT:
            self._cmd_v = 0.0
            self._cmd_w = 0.0
            self._int  *= 0.95
            self._publish(0.0, 0.0)
            return

        # PI: activa cuando cualquier rueda se mueve
        wheel_moving = max(abs(self._v_r), abs(self._v_l)) >= ENC_MIN
        if self._enc_received and wheel_moving:
            expected_diff = w * self.wheel_L / self.wheel_r
            error         = (self._v_r - self._v_l) - expected_diff
            self._int    += error * 0.05   # dt = 20 Hz
            self._int     = max(-self.int_max, min(self.int_max, self._int))
            w_correction  = -(self.kp * error + self.ki * self._int)
            self._diag_correction = w_correction
            self._pi_activations += 1
            w += w_correction
        else:
            self._int *= 0.95

        self._publish(v, w)

    def _publish(self, v: float, w: float):
        out = Twist()
        out.linear.x  = max(-self.max_lin, min(self.max_lin, v))
        out.angular.z = max(-self.max_ang, min(self.max_ang, w))
        self.pub.publish(out)

    # ── Diagnóstico ───────────────────────────────────────────────────────────

    def _diag(self):
        enc_diff = self._v_r - self._v_l
        self.get_logger().info(
            f'[relay] v_r={self._v_r:+.2f} v_l={self._v_l:+.2f} '
            f'diff={enc_diff:+.2f} pi_corr={self._diag_correction:+.3f} '
            f'int={self._int:+.3f} pi_n={self._pi_activations} '
            f'left_scale={self.left_scale:.3f}'
            + (' [SIN ENCODERS]' if not self._enc_received else '')
        )
        self._pi_activations = 0


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
