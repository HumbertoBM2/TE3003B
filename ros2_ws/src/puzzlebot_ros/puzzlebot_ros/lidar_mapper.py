#!/usr/bin/env python3

import math
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def _bresenham(x0, y0, x1, y1):
  
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class LidarMapper(Node):

    def __init__(self):
        super().__init__('lidar_mapper')

        self._res      = float(self.declare_parameter('resolution', 0.05).value)
        self._width_m  = float(self.declare_parameter('width_meters',  4.26).value)
        self._height_m = float(self.declare_parameter('height_meters', 5.36).value)
        self._origin_x = float(self.declare_parameter('origin_x', -0.25).value)
        self._origin_y = float(self.declare_parameter('origin_y', -0.25).value)

        self._cells_x = int(math.ceil(self._width_m  / self._res))   # ~86
        self._cells_y = int(math.ceil(self._height_m / self._res))   # ~108

     
        self._max_range = float(self.declare_parameter('max_mapping_range', 6.0).value)

    

    
        self._lo = np.zeros((self._cells_y, self._cells_x), dtype=np.float32)

  
        self._l_occ   =  math.log(0.80 / 0.20)   
        self._l_free  =  math.log(0.45 / 0.55)   
        self._l_clamp =  5.0                     

   
        self._rx = self._ry = self._ryaw = 0.0
        self._pose_ok = True
        self._scans_processed = 0
        self._odom_received = 0
        self._last_valid_ranges = 0

       
        self._kf_x   = None  
        self._kf_y   = None
        self._kf_yaw = None
        self._KF_MIN_DIST = 0.10    # m
        self._KF_MIN_ROT  = math.radians(5.0)  # rad

      
        self.create_subscription(Odometry,  '/slam/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_corrected', self._scan_cb, qos_profile_sensor_data)

        
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._pub = self.create_publisher(OccupancyGrid, '/map', qos_map)

      
        self.create_service(Trigger, '/map/save',  self._svc_save)
        self.create_service(Trigger, '/map/clear', self._svc_clear)

    
        self.create_timer(0.5, self._publish)

        
        self.create_timer(5.0, self._diag_log)

        self.get_logger().info(
            f'lidar_mapper: {self._cells_x}×{self._cells_y} celdas, '
            f'{self._res}m/celda, área {self._width_m:.2f}m×{self._height_m:.2f}m, '
            f'origen=({self._origin_x:.2f},{self._origin_y:.2f})'
        )

   

    def _diag_log(self):
        cells_known = int(np.sum(np.abs(self._lo) > 0.5))
        if self._scans_processed == 0:
            self.get_logger().warn(
                f'[lidar_mapper] SIN SCANS — scans=0, odom={self._odom_received}'
                f' | ¿lidar encendido? ¿topic /scan publicándose?'
            )
        else:
            self.get_logger().info(
                f'[lidar_mapper] OK — scans={self._scans_processed}'
                f' odom={self._odom_received}'
                f' pose=({self._rx:.2f},{self._ry:.2f},{math.degrees(self._ryaw):.1f}°)'
                f' celdas_conocidas={cells_known}'
                f' últimas_ranges_válidas={self._last_valid_ranges}'
            )

    def _odom_cb(self, msg: Odometry):
        self._odom_received += 1
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._ryaw = math.atan2(siny, cosy)
        self._pose_ok = True

  

    def _w2g(self, wx, wy):
        """Convierte coordenadas mundo (m) a índice de celda (entero)."""
        return (
            int((wx - self._origin_x) / self._res),
            int((wy - self._origin_y) / self._res),
        )

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self._cells_x and 0 <= gy < self._cells_y

    def _scan_cb(self, msg: LaserScan):
        self._scans_processed += 1
        try:
            self._process_scan(msg)
        except Exception as exc:
            self.get_logger().error(f'[lidar_mapper] excepción en scan_cb: {exc}', throttle_duration_sec=5.0)

    def _process_scan(self, msg: LaserScan):
        rx, ry, rth = self._rx, self._ry, self._ryaw

     
        if self._kf_x is None:
            self._kf_x, self._kf_y, self._kf_yaw = rx, ry, rth
        else:
            dist = math.hypot(rx - self._kf_x, ry - self._kf_y)
            drot = abs(math.atan2(math.sin(rth - self._kf_yaw),
                                  math.cos(rth - self._kf_yaw)))
            if dist < self._KF_MIN_DIST and drot < self._KF_MIN_ROT:
                return  
            self._kf_x, self._kf_y, self._kf_yaw = rx, ry, rth

        rx_g, ry_g = self._w2g(rx, ry)
        if not self._in_bounds(rx_g, ry_g):
            self.get_logger().warn(
                f'[lidar_mapper] robot fuera del mapa: ({rx:.2f},{ry:.2f}) → celda ({rx_g},{ry_g})',
                throttle_duration_sec=10.0
            )
            return

        lo      = self._lo
        l_occ   = self._l_occ
        l_free  = self._l_free
        clamp   = self._l_clamp
        angle   = msg.angle_min
        valid   = 0

        for r in msg.ranges:
            a = rth + angle  
            angle += msg.angle_increment

            if not (msg.range_min <= r <= msg.range_max):
                continue
            if r > self._max_range:
                continue  
            valid += 1

            hx_g, hy_g = self._w2g(
                rx + r * math.cos(a),
                ry + r * math.sin(a),
            )

            
            for cx, cy in _bresenham(rx_g, ry_g, hx_g, hy_g):
                if not self._in_bounds(cx, cy):
                    break
                if cx == hx_g and cy == hy_g:
                    break
                v = lo[cy, cx] + l_free
                lo[cy, cx] = v if v > -clamp else -clamp

           
            if self._in_bounds(hx_g, hy_g):
                v = lo[hy_g, hx_g] + l_occ
                lo[hy_g, hx_g] = v if v < clamp else clamp

        self._last_valid_ranges = valid
        if self._scans_processed == 1:
            self.get_logger().info(
                f'[lidar_mapper] PRIMER SCAN recibido — ranges={len(msg.ranges)} válidos={valid}'
                f' robot=({rx:.2f},{ry:.2f})'
            )

  

    def _publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self._res
        msg.info.width  = self._cells_x
        msg.info.height = self._cells_y
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

  
        abs_lo = np.abs(self._lo)
        data = np.full((self._cells_y, self._cells_x), -1, dtype=np.int8)
        known = abs_lo > 0.5
        data[known] = np.where(self._lo[known] > 0, 100, 0).astype(np.int8)
        msg.data = data.flatten().tolist()

        self._pub.publish(msg)

  

    def _svc_save(self, _req, resp):
        try:
            maps_dir = os.path.expanduser('~/maps')
            os.makedirs(maps_dir, exist_ok=True)
            ts   = time.strftime('%Y%m%d_%H%M%S')
            base = os.path.join(maps_dir, f'map_{ts}')
            self._write_pgm_yaml(base)
            self._write_pgm_yaml(os.path.join(maps_dir, 'current'))
            resp.success = True
            resp.message = f'Mapa guardado: {base}.pgm  y  ~/maps/current.pgm'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al guardar mapa: {exc}'
            self.get_logger().error(resp.message)
        return resp

    def _write_pgm_yaml(self, base):
        """Guarda el mapa en formato nav2_map_server (pgm + yaml)."""
        pgm_path  = base + '.pgm'
        yaml_path = base + '.yaml'

     
        img = np.full((self._cells_y, self._cells_x), 205, dtype=np.uint8)
        abs_lo = np.abs(self._lo)
        known  = abs_lo > 0.5
        img[known & (self._lo < 0)] = 254   # libre → blanco
        img[known & (self._lo > 0)] = 0     # ocupado → negro

 
        img = np.flipud(img)

        h, w = img.shape
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(img.tobytes())

        with open(yaml_path, 'w') as f:
            f.write(
                f'image: {os.path.basename(pgm_path)}\n'
                f'resolution: {self._res}\n'
                f'origin: [{self._origin_x}, {self._origin_y}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.196\n'
            )

 

    def _svc_clear(self, _req, resp):
        self._lo.fill(0.0)
     
        self._kf_x = self._kf_y = self._kf_yaw = None
        resp.success = True
        resp.message = 'Mapa reiniciado (todo unknown)'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Guardando mapa antes de salir...')
        try:
            maps_dir = os.path.expanduser('~/maps')
            os.makedirs(maps_dir, exist_ok=True)
            node._write_pgm_yaml(os.path.join(maps_dir, 'current'))
            node.get_logger().info('Mapa guardado en ~/maps/current.pgm')
        except Exception as exc:
            node.get_logger().error(f'Error al guardar mapa: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
