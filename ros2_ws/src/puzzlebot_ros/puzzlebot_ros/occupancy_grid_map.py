"""Occupancy grid log-odds + integración de rayos lidar (de puzzlebot_sim)."""

import math
from typing import Optional

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import LaserScan

from .slam_math import bresenham
from .slam_types import Pose2D


class OccupancyGridMap:
    def __init__(
        self,
        size_pixels: int,
        size_meters: float,
        origin_x: float,
        origin_y: float,
        p_occ: float,
        p_free: float,
        l_clamp: float,
        scan_step: int,
        max_range_factor: float,
        min_useful_range: float,
        max_mapping_range: float,
        lidar_x: float,
        lidar_y: float,
        lidar_yaw: float,
        width_pixels: Optional[int] = None,
        height_pixels: Optional[int] = None,
        resolution: Optional[float] = None,
    ):
        self.size_pixels = size_pixels
        self.size_meters = size_meters
        self.resolution = (
            size_meters / size_pixels if resolution is None else resolution)
        self.width_pixels = size_pixels if width_pixels is None else width_pixels
        self.height_pixels = size_pixels if height_pixels is None else height_pixels
        self.width_meters = self.width_pixels * self.resolution
        self.height_meters = self.height_pixels * self.resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.l_occ = math.log(p_occ / (1.0 - p_occ))
        self.l_free = math.log(p_free / (1.0 - p_free))
        self.l_clamp = l_clamp
        self.scan_step = max(1, scan_step)
        self.max_range_factor = max_range_factor
        self.min_useful_range = min_useful_range
        self.max_mapping_range = max_mapping_range
        self.lidar_x = lidar_x
        self.lidar_y = lidar_y
        self.lidar_yaw = lidar_yaw
        self.grid = np.zeros((self.height_pixels, self.width_pixels), dtype=np.float32)

    def clear(self):
        self.grid.fill(0.0)

    def world_to_cell(self, wx: float, wy: float):
        col = int(math.floor((wx - self.origin_x) / self.resolution))
        row = int(math.floor((wy - self.origin_y) / self.resolution))
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width_pixels and 0 <= row < self.height_pixels

    def integrate_scan(self, scan: LaserScan, pose: Pose2D) -> bool:
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        sensor_x = pose.x + self.lidar_x * c - self.lidar_y * s
        sensor_y = pose.y + self.lidar_x * s + self.lidar_y * c

        r_col, r_row = self.world_to_cell(sensor_x, sensor_y)
        if not self.in_bounds(r_col, r_row):
            return False

        rmin = max(self.min_useful_range, scan.range_min)
        rmax = scan.range_max
        if self.max_mapping_range > 0.0:
            rmax = min(rmax, self.max_mapping_range)
        hit_threshold = rmax * self.max_range_factor

        for index in range(0, len(scan.ranges), self.scan_step):
            r = scan.ranges[index]
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue

            is_hit = r < hit_threshold
            angle = scan.angle_min + index * scan.angle_increment + pose.yaw + self.lidar_yaw
            end_x = sensor_x + r * math.cos(angle)
            end_y = sensor_y + r * math.sin(angle)
            e_col, e_row = self.world_to_cell(end_x, end_y)

            if not self.in_bounds(e_col, e_row):
                e_col = max(0, min(self.width_pixels - 1, e_col))
                e_row = max(0, min(self.height_pixels - 1, e_row))
                is_hit = False

            self._integrate_ray(r_col, r_row, e_col, e_row, is_hit)

        return True

    def _integrate_ray(self, r_col, r_row, e_col, e_row, is_hit) -> None:
        cells = list(bresenham(r_col, r_row, e_col, e_row))
        if not cells:
            return

        for col, row in cells[:-1]:
            if self.in_bounds(col, row):
                self.grid[row, col] = max(
                    -self.l_clamp, self.grid[row, col] + self.l_free)

        end_col, end_row = cells[-1]
        if not self.in_bounds(end_col, end_row):
            return

        if is_hit:
            self.grid[end_row, end_col] = min(
                self.l_clamp, self.grid[end_row, end_col] + self.l_occ)
        else:
            self.grid[end_row, end_col] = max(
                -self.l_clamp, self.grid[end_row, end_col] + self.l_free)

    def to_msg(self, stamp, frame_id: str) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.width_pixels
        msg.info.height = self.height_pixels
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        flat = self.grid.flatten()
        data = np.full(flat.shape, -1, dtype=np.int8)
        data[flat > 0.5] = 100
        data[flat < -0.5] = 0
        msg.data = data.tolist()
        return msg
