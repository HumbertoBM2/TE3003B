#!/usr/bin/env python3

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist


WHEEL_BIAS = 0.80  

KEY_MOVE = {
    'i': ( 1.0,  0.0),
    ',': (-1.0,  0.0),
    'j': ( 0.0,  1.0),
    'l': ( 0.0, -1.0),
    'u': ( 1.0,  1.0),
    'o': ( 1.0, -1.0),
    'm': (-1.0,  1.0),
    '.': (-1.0, -1.0),
    'k': ( 0.0,  0.0),
    ' ': ( 0.0,  0.0),
}


class PuzzlebotTeleop(Node):

    def __init__(self):
        super().__init__('puzzlebot_teleop')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)

        self.speed = 0.15
        self.turn  = 0.8
        self._lin  = 0.0
        self._ang  = 0.0

        self.create_timer(0.05, self._publish)
        self._ready = False
        self._ready_timer = self.create_timer(3.0, self._set_ready)

        self.get_logger().info(
            'puzzlebot_teleop: publicando /cmd_vel BEST_EFFORT — esperando 3s para DDS...'
        )
        print(
            '\n══════════════════════════════════\n'
            '  PUZZLEBOT TELEOP\n'
            '  Espera 3 segundos antes de mover...\n'
            '══════════════════════════════════\n'
        )

    def _set_ready(self):
        self._ready = True
        self._ready_timer.cancel()
        self.get_logger().info('puzzlebot_teleop: LISTO — presiona i para adelante, k para stop')
        print(
            '  LISTO — i=adelante  ,=atrás\n'
            '  j=izquierda  l=derecha  k=STOP\n'
            '  q/z=velocidad+/-\n'
            f'  speed={self.speed:.2f} m/s   turn={self.turn:.2f} rad/s\n'
        )

    def _publish(self):
        msg = Twist()
        v = self._lin * self.speed
        msg.linear.x  = v
        msg.angular.z = self._ang * self.turn + WHEEL_BIAS * v
        self.pub.publish(msg)

    def process_key(self, key):
        if not self._ready:
            return
        if key in KEY_MOVE:
            self._lin, self._ang = KEY_MOVE[key]
        elif key == 'q':
            self.speed = min(self.speed + 0.05, 0.5)
            print(f'speed={self.speed:.2f} m/s')
        elif key == 'z':
            self.speed = max(self.speed - 0.05, 0.05)
            print(f'speed={self.speed:.2f} m/s')
        elif key == 'e':
            self.turn = min(self.turn + 0.1, 2.0)
            print(f'turn={self.turn:.2f} rad/s')
        elif key == 'c':
            self.turn = max(self.turn - 0.1, 0.1)
            print(f'turn={self.turn:.2f} rad/s')
        else:
            self._lin = 0.0
            self._ang = 0.0


def _get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = _get_key()
            if key == '\x03':
                break
            node.process_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node._lin = 0.0
        node._ang = 0.0
        node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
