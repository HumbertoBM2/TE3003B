#!/usr/bin/env python3


import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile, DurabilityPolicy, ReliabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

from .my_math import wrap_to_pi, quaternion_from_euler

WARMUP_SCANS = 15     

COARSE_HALF  = math.radians(8.0)
COARSE_STEP  = math.radians(2.0)
FINE_HALF    = math.radians(1.5)
FINE_STEP    = math.radians(0.5)

TRANS_HALF   = 0.12   
TRANS_STEP   = 0.04     

RAY_STRIDE         = 4     
MAX_SCORING_RANGE  = 5.5   
MIN_SCORE          = 5.0   
MIN_SCORE_TRANS    = 4.0    
MAX_CORR_XY        = 0.18  
MAX_CORR_TH        = math.radians(10.0)
MIN_MAP_CELLS      = 200   


class ScanMatcher(Node):

    def __init__(self):
        super().__init__('scan_matcher')

        self._ekf_x  = 0.0
        self._ekf_y  = 0.0
        self._ekf_th = 0.0
        self._ekf_ok = False

        self._grid      = None   
        self._map_ox    = 0.0   
        self._map_oy    = 0.0    
        self._map_res   = 0.05   
        self._map_w     = 0
        self._map_h     = 0
        self._map_cells = 0      

        self._scan_count  = 0
        self._ok          = 0
        self._fail        = 0
        self._last_score  = 0.0


        qos_map = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Odometry, '/slam/odom',
                                 self._ekf_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map',
                                 self._map_cb, qos_map)
        self.create_subscription(LaserScan, '/scan_corrected',
                                 self._scan_cb, qos_profile_sensor_data)

        self.pub_delta = self.create_publisher(Odometry, '/scan_match/delta', 10)
        self.pub_pose  = self.create_publisher(Odometry, '/scan_match/pose',  10)
        self.create_timer(5.0, self._diag)

        self.get_logger().info('scan_matcher (scan-to-map grid scoring) iniciado')


    def _ekf_cb(self, msg: Odometry):
        self._ekf_x = msg.pose.pose.position.x
        self._ekf_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._ekf_th = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._ekf_ok = True

    def _map_cb(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self._grid = np.where(data > 50, data.astype(np.float32), 0.0)
        self._map_ox  = msg.info.origin.position.x
        self._map_oy  = msg.info.origin.position.y
        self._map_res = msg.info.resolution
        self._map_w   = msg.info.width
        self._map_h   = msg.info.height
        self._map_cells = int(np.sum(data != -1))


    def _scan_cb(self, msg: LaserScan):
        self._scan_count += 1

        if self._grid is None or not self._ekf_ok:
            return

        if self._scan_count <= WARMUP_SCANS or self._map_cells < MIN_MAP_CELLS:
            return

        ranges, rel_angles = self._valid_rays(msg)
        if len(ranges) < 20:
            return

        ix, iy, ith = self._ekf_x, self._ekf_y, self._ekf_th

        best_yaw   = ith
        best_score = -1.0

        for dyaw in np.arange(-COARSE_HALF, COARSE_HALF + 1e-9, COARSE_STEP):
            s = self._score(ranges, rel_angles, ix, iy, ith + dyaw)
            if s > best_score:
                best_score = s
                best_yaw   = ith + dyaw

        coarse_yaw = best_yaw
        for dyaw in np.arange(-FINE_HALF, FINE_HALF + 1e-9, FINE_STEP):
            yaw = coarse_yaw + dyaw
            s   = self._score(ranges, rel_angles, ix, iy, yaw)
            if s > best_score:
                best_score = s
                best_yaw   = yaw

        mx, my, mth = ix, iy, best_yaw
        rot_score   = best_score

        if rot_score >= MIN_SCORE_TRANS:
            offsets = np.arange(-TRANS_HALF, TRANS_HALF + 1e-9, TRANS_STEP)
            for dx in offsets:
                for dy in offsets:
                    s = self._score(ranges, rel_angles,
                                    ix + dx, iy + dy, mth)
                    if s > best_score:
                        best_score = s
                        mx, my    = ix + dx, iy + dy

        self._last_score = best_score

        if best_score < MIN_SCORE:
            self._fail += 1
            return

        corr_xy = math.hypot(mx - ix, my - iy)
        corr_th = abs(wrap_to_pi(mth - ith))
        if corr_xy > MAX_CORR_XY or corr_th > MAX_CORR_TH:
            self.get_logger().warn(
                f'Corrección grande rechazada ({corr_xy:.2f}m, '
                f'{math.degrees(corr_th):.1f}°)',
                throttle_duration_sec=2.0)
            self._fail += 1
            return

        self._ok += 1

        dx_map = mx - ix
        dy_map = my - iy
        dth    = wrap_to_pi(mth - ith)
        c = math.cos(-ith)
        s = math.sin(-ith)
        dx_robot = dx_map * c - dy_map * s
        dy_robot = dx_map * s + dy_map * c

        now = msg.header.stamp

        delta = Odometry()
        delta.header.stamp    = now
        delta.header.frame_id = 'base_link'
        delta.child_frame_id  = 'base_link'
        delta.pose.pose.position.x = dx_robot
        delta.pose.pose.position.y = dy_robot
        delta.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, dth)
        delta.pose.covariance[0]  = max(1e-4, 0.01 * corr_xy)
        delta.pose.covariance[7]  = max(1e-4, 0.01 * corr_xy)
        delta.pose.covariance[35] = max(5e-5, 0.005 * corr_th)
        self.pub_delta.publish(delta)

        pose_msg = Odometry()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'map'
        pose_msg.child_frame_id  = 'base_link'
        pose_msg.pose.pose.position.x = mx
        pose_msg.pose.pose.position.y = my
        pose_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, mth)
        self.pub_pose.publish(pose_msg)


    def _valid_rays(self, msg: LaserScan):
        n      = len(msg.ranges)
        r_all  = np.asarray(msg.ranges, dtype=np.float32)
        a_all  = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        rmin   = max(float(msg.range_min), 0.15)
        rmax   = min(float(msg.range_max) * 0.95, MAX_SCORING_RANGE)
        valid  = np.isfinite(r_all) & (r_all > rmin) & (r_all < rmax)
        idx    = np.where(valid)[0][::RAY_STRIDE]
        return r_all[idx], a_all[idx]

    def _score(self, ranges: np.ndarray, rel_angles: np.ndarray,
               px: float, py: float, yaw: float) -> float:
        wx = px + ranges * np.cos(rel_angles + yaw)
        wy = py + ranges * np.sin(rel_angles + yaw)

        col = ((wx - self._map_ox) / self._map_res).astype(np.int32)
        row = ((wy - self._map_oy) / self._map_res).astype(np.int32)

        mask = (col >= 0) & (col < self._map_w) & (row >= 0) & (row < self._map_h)
        if not np.any(mask):
            return 0.0

        return float(np.sum(self._grid[row[mask], col[mask]]))

    def _diag(self):
        self.get_logger().info(
            f'[scan_match] ok={self._ok} fail={self._fail} '
            f'score={self._last_score:.1f} '
            f'map_cells={self._map_cells} '
            f'scans={self._scan_count} '
            f'ekf={"si" if self._ekf_ok else "no"}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcher()
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
