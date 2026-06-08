#!/usr/bin/env python3


import math
from collections import deque
from math import atan2, cos, degrees, pi, sin, sqrt

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from .my_math import euler_from_quaternion, wrap_to_pi

N_PARTICLES    = 100     
SCAN_STEP      = 10     
SIGMA          = 0.20   
INIT_SXY       = 0.10    
INIT_SYAW      = 0.08   
REINJECT_FRAC  = 0.30    
ALPHA_TRANS    = 0.10    
ALPHA_ROT      = 0.08   
NOISE_FLOOR_XY = 0.003  
NOISE_FLOOR_YAW= 0.003  


class ParticleLocalization(Node):

    def __init__(self):
        super().__init__('particle_localization')

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self._pub_mo  = self.create_publisher(TransformStamped, '/pf/map_to_odom',  qos_rel)
        self._pub_pts = self.create_publisher(PoseArray,         '/pf/particles',    qos_rel)

        self.create_subscription(OccupancyGrid,    '/map',          self._map_cb,    qos_latched)
        self.create_subscription(Odometry,         '/odom',         self._odom_cb,   qos_be)
        self.create_subscription(LaserScan,        '/scan_stamped', self._scan_cb,   qos_be)
        self.create_subscription(TransformStamped, '/map_to_odom',  self._aruco_cb,  qos_rel)

    
        self._p: np.ndarray     = None
        self._w: np.ndarray     = np.ones(N_PARTICLES) / N_PARTICLES

        self._map_info  = None
        self._dist_field: np.ndarray = None   

        self._odom_prev = None   

        self._est_x   = 0.0
        self._est_y   = 0.0
        self._est_yaw = 0.0
        self._ok      = False     

        self.create_timer(2.0, self._diag)
        self.get_logger().info(f'Filtro de particulas MCL listo — N={N_PARTICLES}')


    def _map_cb(self, msg: OccupancyGrid):
        self._map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self._dist_field = self._build_dist_field(grid, msg.info.resolution)
        self.get_logger().info(
            f'Campo de distancias precomputado: {msg.info.width}x{msg.info.height}')

    def _aruco_cb(self, msg: TransformStamped):
        """Corrección ArUco: inicializa o reinyecta partículas.

        /map_to_odom es el TF map→odom (origen del frame odom en frame map),
        NO la pose del robot.  La pose del robot en frame map es:
            rx = mo_x + cos(mo_yaw)*odom_x - sin(mo_yaw)*odom_y
            ry = mo_y + sin(mo_yaw)*odom_x + cos(mo_yaw)*odom_y
        """
        if self._odom_prev is None:
            return 

        mo_x   = msg.transform.translation.x
        mo_y   = msg.transform.translation.y
        q      = msg.transform.rotation
        mo_yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        ox, oy, oyaw = self._odom_prev
        c  = cos(mo_yaw)
        s  = sin(mo_yaw)
        rx   = mo_x + c * ox - s * oy
        ry   = mo_y + s * ox + c * oy
        ryaw = wrap_to_pi(oyaw + mo_yaw)

        if not self._ok:
         
            if sqrt(rx * rx + ry * ry) < 0.3:
                return
            self._init(rx, ry, ryaw)
            return

        err = sqrt((self._est_x - rx) ** 2 + (self._est_y - ry) ** 2)
        if err > 1.5:
            self.get_logger().warn(
                f'PF: divergencia {err:.2f}m — reinit en ({rx:.2f},{ry:.2f})')
            self._init(rx, ry, ryaw)
            return

     
        n = max(1, int(N_PARTICLES * REINJECT_FRAC))
        low = np.argsort(self._w)[:n]
        self._p[low, 0] = rx   + np.random.randn(n) * INIT_SXY   * 0.5
        self._p[low, 1] = ry   + np.random.randn(n) * INIT_SXY   * 0.5
        self._p[low, 2] = ryaw + np.random.randn(n) * INIT_SYAW  * 0.5
        self._w[low]    = 1.0 / N_PARTICLES
        self._w        /= self._w.sum()

    def _odom_cb(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        _, _, oyaw = euler_from_quaternion(msg.pose.pose.orientation)

        if self._odom_prev is None:
            self._odom_prev = (ox, oy, oyaw)
            return
        if not self._ok:
            self._odom_prev = (ox, oy, oyaw)
            return

        px, py, pyaw = self._odom_prev

        dxr  =  cos(pyaw) * (ox - px) + sin(pyaw) * (oy - py)
        dyr  = -sin(pyaw) * (ox - px) + cos(pyaw) * (oy - py)
        dyaw = wrap_to_pi(oyaw - pyaw)

        move = sqrt(dxr*dxr + dyr*dyr)

        if move > 1e-4 or abs(dyaw) > 5e-4:
            self._motion_update(dxr, dyr, dyaw, move)

        self._odom_prev = (ox, oy, oyaw)

    def _scan_cb(self, msg: LaserScan):
        if not self._ok or self._dist_field is None:
            return

        raw_r = np.array(msg.ranges[::SCAN_STEP], dtype=np.float32)
        n_r   = len(raw_r)
        angs  = (msg.angle_min + np.arange(n_r) * msg.angle_increment * SCAN_STEP
                 ).astype(np.float32)

        valid  = np.isfinite(raw_r) & (raw_r > 0.10) & (raw_r < 6.0)
        ranges = raw_r[valid]
        angles = angs[valid]

        if len(ranges) < 5:
            return

        self._obs_update(ranges, angles)

        ess = 1.0 / float(np.sum(self._w ** 2) + 1e-12)
        if ess < N_PARTICLES * 0.5:
            self._resample()

        self._est_x   = float(np.average(self._p[:, 0], weights=self._w))
        self._est_y   = float(np.average(self._p[:, 1], weights=self._w))
  
        sin_w = float(np.dot(self._w, np.sin(self._p[:, 2])))
        cos_w = float(np.dot(self._w, np.cos(self._p[:, 2])))
        self._est_yaw = math.atan2(sin_w, cos_w)

        self._publish_mo()
        self._publish_particles()


    def _motion_update(self, dxr: float, dyr: float, dyaw: float, dist: float):
        """Rotación delta al frame de cada partícula + ruido gaussiano."""
        N    = N_PARTICLES
        s_xy = dist  * ALPHA_TRANS + NOISE_FLOOR_XY
        s_yw = abs(dyaw) * ALPHA_ROT + NOISE_FLOOR_YAW

        c_p = np.cos(self._p[:, 2])
        s_p = np.sin(self._p[:, 2])

        self._p[:, 0] += c_p * dxr - s_p * dyr + np.random.randn(N) * s_xy
        self._p[:, 1] += s_p * dxr + c_p * dyr + np.random.randn(N) * s_xy
        self._p[:, 2] += dyaw + np.random.randn(N) * s_yw
        self._p[:, 2]  = (self._p[:, 2] + pi) % (2 * pi) - pi  # normalizar


    def _obs_update(self, ranges: np.ndarray, angles: np.ndarray):
   
        info  = self._map_info
        res   = info.resolution
        ox    = info.origin.position.x
        oy    = info.origin.position.y
        h, w  = self._dist_field.shape
        s2    = 2.0 * SIGMA * SIGMA

        log_w = np.zeros(N_PARTICLES, dtype=np.float64)

        for r, a in zip(ranges, angles):
            wx = self._p[:, 0] + r * np.cos(self._p[:, 2] + a)
            wy = self._p[:, 1] + r * np.sin(self._p[:, 2] + a)

            cols = np.clip(((wx - ox) / res).astype(np.int32), 0, w - 1)
            rows = np.clip(((wy - oy) / res).astype(np.int32), 0, h - 1)

            d    = self._dist_field[rows, cols]  
            log_w += -(d * d) / s2

        log_w -= log_w.max()
        w_obs  = np.exp(log_w)

        self._w *= w_obs
        total   = self._w.sum()
        if total > 1e-12:
            self._w /= total
        else:
            self._w = np.ones(N_PARTICLES) / N_PARTICLES


    def _resample(self):
        N   = N_PARTICLES
        pos = (np.arange(N) + np.random.uniform()) / N
        cum = np.cumsum(self._w)
        idx = np.clip(np.searchsorted(cum, pos), 0, N - 1)
        self._p  = self._p[idx].copy()
        self._w  = np.ones(N) / N


    @staticmethod
    def _build_dist_field(grid: np.ndarray, res: float) -> np.ndarray:
        occupied = (grid > 50)
        try:
            from scipy.ndimage import distance_transform_edt
            dist_px = distance_transform_edt(~occupied)
        except ImportError:
            dist_px = ParticleLocalization._bfs_dist(occupied)
        return (dist_px * res).astype(np.float32)

    @staticmethod
    def _bfs_dist(occupied: np.ndarray) -> np.ndarray:
        h, w   = occupied.shape
        dist   = np.full((h, w), 1e6, dtype=np.float32)
        queue  = deque()
        ys, xs = np.where(occupied)
        for y, x in zip(ys, xs):
            dist[y, x] = 0.0
            queue.append((y, x))
        dirs  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        costs = [1.0,   1.0,  1.0,   1.0,   1.414,  1.414,  1.414,  1.414]
        while queue:
            cy, cx = queue.popleft()
            for (dy, dx), cost in zip(dirs, costs):
                ny, nx = cy+dy, cx+dx
                if 0 <= ny < h and 0 <= nx < w:
                    nd = dist[cy, cx] + cost
                    if nd < dist[ny, nx]:
                        dist[ny, nx] = nd
                        queue.append((ny, nx))
        return dist


    def _init(self, x: float, y: float, yaw: float):
        N = N_PARTICLES
        self._p = np.zeros((N, 3), dtype=np.float64)
        self._p[:, 0] = x   + np.random.randn(N) * INIT_SXY
        self._p[:, 1] = y   + np.random.randn(N) * INIT_SXY
        self._p[:, 2] = yaw + np.random.randn(N) * INIT_SYAW
        self._w  = np.ones(N) / N
        self._est_x   = x
        self._est_y   = y
        self._est_yaw = yaw
        self._ok      = True
        self.get_logger().info(
            f'Partículas inicializadas: ({x:.2f}, {y:.2f}, {degrees(yaw):.1f}°)')


    def _publish_mo(self):
        if self._odom_prev is None:
            return
        ox, oy, oyaw = self._odom_prev

        mo_yaw = wrap_to_pi(self._est_yaw - oyaw)
        c, s   = cos(mo_yaw), sin(mo_yaw)
        mo_x   = self._est_x - (c * ox - s * oy)
        mo_y   = self._est_y - (s * ox + c * oy)

        msg = TransformStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'odom'
        msg.transform.translation.x = mo_x
        msg.transform.translation.y = mo_y
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z    = sin(mo_yaw / 2.0)
        msg.transform.rotation.w    = cos(mo_yaw / 2.0)
        self._pub_mo.publish(msg)

    def _publish_particles(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for px, py, pyaw in self._p:
            pose = Pose()
            pose.position.x    = float(px)
            pose.position.y    = float(py)
            pose.orientation.z = sin(float(pyaw) / 2.0)
            pose.orientation.w = cos(float(pyaw) / 2.0)
            msg.poses.append(pose)
        self._pub_pts.publish(msg)

    def _diag(self):
        if not self._ok:
            return
        ess = 1.0 / float(np.sum(self._w ** 2) + 1e-12)
        self.get_logger().info(
            f'[PF] est=({self._est_x:.2f},{self._est_y:.2f},{degrees(self._est_yaw):.1f}°) '
            f'ESS={ess:.0f}/{N_PARTICLES}')


def main(args=None):
    rclpy.init(args=args)
    node = ParticleLocalization()
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
