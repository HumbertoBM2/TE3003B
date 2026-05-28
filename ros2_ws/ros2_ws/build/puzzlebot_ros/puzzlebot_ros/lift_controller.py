#!/usr/bin/env python3
"""
Lift controller — servo de elevación via GPIO del Jetson → FPGA Tang Nano 20K.

Suscribe /lift/command (std_msgs/String): "UP", "DOWN", "STOP"

GPIO sysfs:
  GPIO168 (pin físico 32) → pb1 del FPGA (pin 15 del Tang Nano)
  GPIO38  (pin físico 33) → pb2 del FPGA (pin 16 del Tang Nano)

Señales:
  pb1=1, pb2=0 → servo UP   (pulso ~2.48ms)
  pb1=0, pb2=1 → servo DOWN (pulso ~0.56ms)
  pb1=0, pb2=0 → NEUTRO     (pulso 1.5ms)

Prerequisito (una vez):
  sudo usermod -a -G gpio puzzlebot   # o correr nodo con sudo
"""
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_GPIO_PB1 = 168   # physical pin 32
_GPIO_PB2 = 38    # physical pin 33


def _gpio_path(pin, name=''):
    base = f'/sys/class/gpio/gpio{pin}'
    return os.path.join(base, name) if name else base


def _export(pin):
    if not os.path.exists(_gpio_path(pin)):
        with open('/sys/class/gpio/export', 'w') as f:
            f.write(str(pin))



def _set_direction(pin, direction):
    with open(_gpio_path(pin, 'direction'), 'w') as f:
        f.write(direction)


def _write(pin, val):
    with open(_gpio_path(pin, 'value'), 'w') as f:
        f.write('1' if val else '0')


class LiftController(Node):

    def __init__(self):
        super().__init__('lift_controller')
        self._ok = self._init_gpio()
        self.create_subscription(String, '/lift/command', self._cmd_cb, 10)
        if self._ok:
            self.get_logger().info(
                'Lift controller OK — GPIO168(pb1) GPIO38(pb2)'
            )
        else:
            self.get_logger().error(
                'GPIO no disponible — verifica permisos o grupo gpio'
            )

    def _init_gpio(self):
        try:
            for pin in (_GPIO_PB1, _GPIO_PB2):
                _export(pin)
                _set_direction(pin, 'out')
                _write(pin, 0)
            return True
        except Exception as e:
            self.get_logger().error(f'Error init GPIO: {e}')
            return False

    def _set(self, pb1, pb2):
        if not self._ok:
            return
        try:
            _write(_GPIO_PB1, pb1)
            _write(_GPIO_PB2, pb2)
        except Exception as e:
            self.get_logger().error(f'Error GPIO write: {e}')

    def _cmd_cb(self, msg: String):
        cmd = msg.data.upper().strip()
        if cmd == 'UP':
            self._set(1, 0)
            self.get_logger().info('Lift → UP')
        elif cmd == 'DOWN':
            self._set(0, 1)
            self.get_logger().info('Lift → DOWN')
        else:
            self._set(0, 0)
            self.get_logger().info('Lift → STOP')

    def destroy_node(self):
        self._set(0, 0)   # dejar servo en neutro al salir
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
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
