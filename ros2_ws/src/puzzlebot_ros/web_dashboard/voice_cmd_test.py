#!/usr/bin/env python3


import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String


COMMANDS = {
    'avanza':    {'v':  0.08, 'w':  0.0,  'dur': 0.8,  'lift': None,   'desc': 'Avanzar 0.8 s'},
    'retrocede': {'v': -0.08, 'w':  0.0,  'dur': 0.8,  'lift': None,   'desc': 'Retroceder 0.8 s'},
    'derecha':   {'v':  0.0,  'w': -0.6,  'dur': 0.5,  'lift': None,   'desc': 'Girar derecha 0.5 s'},
    'izquierda': {'v':  0.0,  'w':  0.6,  'dur': 0.5,  'lift': None,   'desc': 'Girar izquierda 0.5 s'},
    'alto':      {'v':  0.0,  'w':  0.0,  'dur': 0.0,  'lift': None,   'desc': 'Detener'},
    'empieza':   {'v':  0.08, 'w':  0.0,  'dur': 1.0,  'lift': None,   'desc': 'Avanzar 1.0 s'},
    'sube':      {'v':  0.0,  'w':  0.0,  'dur': 1.5,  'lift': 'UP',   'desc': 'Lift UP 1.5 s'},
    'baja':      {'v':  0.0,  'w':  0.0,  'dur': 1.5,  'lift': 'DOWN', 'desc': 'Lift DOWN 1.5 s'},
    'gira':      {'v':  0.0,  'w':  0.5,  'dur': 1.5,  'lift': None,   'desc': 'Rotar 1.5 s'},
    'busca':     {'v':  0.0,  'w':  0.4,  'dur': 2.0,  'lift': None,   'desc': 'Rotacion de busqueda 2.0 s'},
}


class VoiceCmdTest(Node):
    def __init__(self):
        super().__init__('voice_cmd_test')

        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # Publishers
        self._pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',       qos_be)
        self._pub_lift   = self.create_publisher(String, '/lift/command',   qos_rel)
        self._pub_action = self.create_publisher(String, '/voice/action',   qos_rel)

        # Subscriber
        self.create_subscription(
            String, '/voice/recognized_command', self._voice_cb, qos_rel)

     
        self._stop_timer = None
        self._stop_lock  = threading.Lock()

        self.get_logger().info('voice_cmd_test listo — escuchando /voice/recognized_command')
        self.get_logger().info(
            'Comandos: ' + ', '.join(COMMANDS.keys()) + ', ninguna (sin accion)')

    def _voice_cb(self, msg: String):
        word = msg.data.strip().lower()

        if word == 'ninguna' or word not in COMMANDS:
            self.get_logger().info(f'"{word}" — sin accion')
            return

        cmd = COMMANDS[word]
        self.get_logger().info(f'"{word}" → {cmd["desc"]}')

     
        with self._stop_lock:
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None

     
        if cmd['v'] != 0.0 or cmd['w'] != 0.0:
            self._send_cmd(cmd['v'], cmd['w'])

    
        if cmd['lift'] is not None:
            self._send_lift(cmd['lift'])

      
        action_msg      = String()
        action_msg.data = cmd['desc']
        self._pub_action.publish(action_msg)

       
        if cmd['dur'] > 0.0:
            with self._stop_lock:
                self._stop_timer = threading.Timer(cmd['dur'], self._stop_all)
                self._stop_timer.daemon = True
                self._stop_timer.start()
        else:
         
            self._stop_all()

    def _stop_all(self):
        self._send_cmd(0.0, 0.0)
        self._send_lift('STOP')

    def _send_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self._pub_cmd.publish(msg)

    def _send_lift(self, cmd: str):
        msg      = String()
        msg.data = cmd
        self._pub_lift.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
     
        if rclpy.ok():
            stop = Twist()
            node._pub_cmd.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
