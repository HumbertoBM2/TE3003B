

import numpy as np

from sensor_msgs.msg import LaserScan

from .occupancy_grid_map import OccupancyGridMap
from .slam_types import Pose2D

_WARMUP_SCANS = 12

_COARSE_HALF_RAD = 0.262    
_COARSE_STEP_RAD = 0.0349  
_FINE_HALF_RAD   = 0.0262  
_FINE_STEP_RAD   = 0.00873 

_TRANS_HALF_M  = 0.20       
_TRANS_STEP_M  = 0.05
_MIN_SCORE_FOR_TRANS = 4.0
_RAY_STRIDE = 3


class LocalScanMatcher:
    def __init__(self, enabled: bool = False):
        self._enabled    = enabled
        self._scan_count = 0
        self._last_score = 0.0

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def last_score(self) -> float:
        return self._last_score

    def match(self, scan: LaserScan, initial_pose: Pose2D,
              grid_map: OccupancyGridMap) -> Pose2D:
        if not self._enabled:
            self._last_score = 0.0
            return initial_pose

        self._scan_count += 1
        if self._scan_count <= _WARMUP_SCANS:
            self._last_score = 0.0
            return initial_pose

        return self._search(scan, initial_pose, grid_map)

    def _search(self, scan, initial_pose, grid_map):
        ranges, rel_angles = self._valid_rays(scan, grid_map)
        if len(ranges) == 0:
            return initial_pose

        best_pose  = initial_pose
        best_score = -1.0

        for dyaw in np.arange(-_COARSE_HALF_RAD, _COARSE_HALF_RAD + 1e-9, _COARSE_STEP_RAD):
            c = Pose2D(initial_pose.x, initial_pose.y, initial_pose.yaw + dyaw)
            sc = self._score(ranges, rel_angles, c, grid_map)
            if sc > best_score:
                best_score, best_pose = sc, c

        coarse_yaw = best_pose.yaw
        for yaw in np.arange(coarse_yaw - _FINE_HALF_RAD, coarse_yaw + _FINE_HALF_RAD + 1e-9, _FINE_STEP_RAD):
            c = Pose2D(initial_pose.x, initial_pose.y, yaw)
            sc = self._score(ranges, rel_angles, c, grid_map)
            if sc > best_score:
                best_score, best_pose = sc, c

        rot_pose  = best_pose
        rot_score = best_score

        if rot_score < _MIN_SCORE_FOR_TRANS:
            self._last_score = rot_score
            return rot_pose

        offsets = np.arange(-_TRANS_HALF_M, _TRANS_HALF_M + 1e-9, _TRANS_STEP_M)
        trans_pose  = rot_pose
        trans_score = rot_score
        for dx in offsets:
            for dy in offsets:
                c = Pose2D(rot_pose.x + dx, rot_pose.y + dy, rot_pose.yaw)
                sc = self._score(ranges, rel_angles, c, grid_map)
                if sc > trans_score:
                    trans_score, trans_pose = sc, c

        self._last_score = trans_score
        return trans_pose

    @staticmethod
    def _valid_rays(scan: LaserScan, grid_map: OccupancyGridMap):
        ranges = np.array(scan.ranges, dtype=np.float32)
        n      = len(ranges)
        angles = (scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment)

        rmin = max(float(scan.range_min), grid_map.min_useful_range)
        rmax = float(scan.range_max)
        if grid_map.max_mapping_range > 0.0:
            rmax = min(rmax, grid_map.max_mapping_range)
        rmax *= grid_map.max_range_factor

        valid = np.isfinite(ranges) & (ranges > rmin) & (ranges < rmax)
        idx   = np.where(valid)[0][::_RAY_STRIDE]
        return ranges[idx], angles[idx]

    @staticmethod
    def _score(ranges, rel_angles, pose, grid_map):
        c = np.cos(pose.yaw)
        s = np.sin(pose.yaw)
        sensor_x = pose.x + grid_map.lidar_x * c - grid_map.lidar_y * s
        sensor_y = pose.y + grid_map.lidar_x * s + grid_map.lidar_y * c

        world_angles = rel_angles + pose.yaw + grid_map.lidar_yaw
        wx = sensor_x + ranges * np.cos(world_angles)
        wy = sensor_y + ranges * np.sin(world_angles)

        res = grid_map.resolution
        col = ((wx - grid_map.origin_x) / res).astype(np.int32)
        row = ((wy - grid_map.origin_y) / res).astype(np.int32)
        width = grid_map.width_pixels
        height = grid_map.height_pixels

        mask = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        if not np.any(mask):
            return 0.0

        return float(np.sum(np.maximum(0.0, grid_map.grid[row[mask], col[mask]])))
