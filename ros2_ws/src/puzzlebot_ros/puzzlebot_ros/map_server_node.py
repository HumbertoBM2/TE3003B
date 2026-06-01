#!/usr/bin/env python3


import math
import os
import struct

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger


class MapServerNode(Node):
    def __init__(self):
        super().__init__('map_server_node')

        self.declare_parameter('map_path', os.path.expanduser('~/maps/current'))

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._pub_map = self.create_publisher(OccupancyGrid, '/map', qos_latched)
        self.create_service(Trigger, '/map_server/reload', self._reload_srv)

        self._load_and_publish()

    def _load_and_publish(self) -> bool:
        map_path = self.get_parameter('map_path').value
        pgm_path = map_path + '.pgm'
        yaml_path = map_path + '.yaml'

        if not os.path.exists(pgm_path):
            self.get_logger().error(f'PGM no encontrado: {pgm_path}')
            return False
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'YAML no encontrado: {yaml_path}')
            return False

        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        resolution       = float(meta['resolution'])
        origin           = meta['origin']               # [x, y, yaw]
        negate           = int(meta.get('negate', 0))
        occupied_thresh  = float(meta.get('occupied_thresh', 0.65))
        free_thresh      = float(meta.get('free_thresh', 0.196))

        w, h, maxval, pixels = self._read_pgm(pgm_path)
        if pixels is None:
            return False

      
        rows = [pixels[i * w : (i + 1) * w] for i in range(h)]
        rows.reverse()                    
        pixels_ros = [p for row in rows for p in row]

        data = []
        for pix in pixels_ros:
            if negate:
                pix = maxval - pix
          
            prob_occ = 1.0 - pix / maxval
            if prob_occ > occupied_thresh:
                data.append(100)
            elif prob_occ < free_thresh:
                data.append(0)
            else:
                data.append(-1)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width  = w
        msg.info.height = h
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0

        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation.z = math.sin(yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(yaw / 2.0)

        msg.data = data

        self._pub_map.publish(msg)
        self.get_logger().info(
            f'Mapa publicado: {w}x{h} px | res={resolution} m/px | '
            f'origen=({origin[0]:.3f},{origin[1]:.3f}) | '
            f'ocupadas={data.count(100)} celdas'
        )
        return True

    def _read_pgm(self, path: str):
        """Lee un archivo PGM binario (P5) o ASCII (P2).
        Devuelve (width, height, maxval, pixels_list) o (0,0,0,None) si falla."""
        try:
            with open(path, 'rb') as f:
                magic = f.readline().decode('ascii').strip()
                if magic not in ('P5', 'P2'):
                    self.get_logger().error(f'PGM magic desconocido: {magic}')
                    return 0, 0, 0, None

                # Saltar comentarios
                line = f.readline().decode('ascii').strip()
                while line.startswith('#') or line == '':
                    line = f.readline().decode('ascii').strip()

                w, h = map(int, line.split())
                maxval = int(f.readline().decode('ascii').strip())

                if magic == 'P5':
                    raw = f.read()
                    if maxval < 256:
                        pixels = list(raw)
                    else:
                        pixels = list(struct.unpack(f'>{w*h}H', raw))
                else:
                    raw = f.read().decode('ascii')
                    pixels = list(map(int, raw.split()))

            return w, h, maxval, pixels
        except Exception as e:
            self.get_logger().error(f'Error leyendo PGM: {e}')
            return 0, 0, 0, None

    def _reload_srv(self, request, response):
        ok = self._load_and_publish()
        response.success = ok
        response.message = 'Mapa recargado' if ok else 'Error al cargar mapa'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
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
