#!/usr/bin/env python3
"""


Published topics:
  /particle_cloud  (geometry_msgs/PoseArray)   — all N particles.
  /mcl_pose        (geometry_msgs/PoseStamped) — best-scoring particle.

Subscribed topics:
  /odom  (nav_msgs/Odometry)
  /scan  (sensor_msgs/LaserScan)

Map conventions (room_map.png):
  Resolution : 0.01 m / pixel
  Origin     : (-3.0, -3.0) in world coordinates
  White (255): free space
  Black   (0): wall / obstacle
  Size       : 600 × 600 px  →  6 m × 6 m
  Free zone  : world x,y ∈ [-2.5, +2.5]  (pixel rows 49-549, cols 50-550)
"""

import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MCLNode(Node):
    """Particle-filter localization node."""

    # ------------------------------------------------------------------
    # Map parameters — must match room_map.png / room_map.yaml
    # ------------------------------------------------------------------
    MAP_RESOLUTION = 0.01   # metres per pixel
    MAP_ORIGIN_X   = -3.0   # world X at pixel column 0
    MAP_ORIGIN_Y   = -3.0   # world Y at pixel row (height-1)  [row=0 → max Y]

    # ------------------------------------------------------------------
    # Filter parameters (tune as needed)
    # ------------------------------------------------------------------
    N_PARTICLES   = 800     # number of particles
    BEAM_STRIDE   = 8       # use every N-th laser beam (360/8 ≈ 45 beams)
    MAX_RANGE     = 4.5     # metres — clamp inf / very-long ranges
    NOISE_XY      = 0.03    # std-dev of position noise added at resample (m)
    NOISE_THETA   = 0.03    # std-dev of heading noise added at resample (rad)
    MOTION_NOISE_XY    = 0.005   # std-dev of noise applied during propagation (m)
    MOTION_NOISE_THETA = 0.005   # (rad)
    # Likelihood field: Gaussian sigma for beam-endpoint scoring
    LIDAR_SIGMA   = 0.10    # metres — width of Gaussian around wall pixels

    def __init__(self):
        super().__init__('mcl_node')

        # Step B: load map
        self._load_map()

        # Step D: initialise particles uniformly over free space
        self._particles = self._init_particles()

        # Odometry state
        self._prev_odom = None
        self._last_odom = None

        # Publishers
        self._pub_particles = self.create_publisher(PoseArray,    '/particle_cloud', 10)
        self._pub_pose      = self.create_publisher(PoseStamped,  '/mcl_pose',       10)

        # Subscribers
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.get_logger().info(
            f'MCL node ready — {self.N_PARTICLES} particles, '
            f'beam stride {self.BEAM_STRIDE}')

    # ------------------------------------------------------------------
    # Step B: map loading
    # ------------------------------------------------------------------
    def _load_map(self):
        map_path = os.path.join(
            get_package_share_directory('mcl_puzzlebot'), 'maps', 'room_map.png')
        self._map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self._map_img is None:
            raise RuntimeError(f'Cannot load map: {map_path}')
        self._map_h, self._map_w = self._map_img.shape

        # Pre-compute free-space pixel list for particle initialisation (step D)
        free_rows, free_cols = np.where(self._map_img > 128)   # OpenCV: row, col
        self._free_pixels = np.column_stack([free_cols, free_rows])  # (K, 2): [col, row]

        # Build Gaussian likelihood field (Thrun §6.4):
        #   distanceTransform gives, for each pixel, the distance (in pixels)
        #   to the nearest WALL (dark) pixel.  We then apply a Gaussian so that
        #   pixels ON the wall surface score ~1.0 and pixels far inside free
        #   space score ~0.  This tolerates 1-2 pixel alignment errors and is
        #   far more robust than direct pixel-value lookup.
        src = (self._map_img > 128).astype(np.uint8)  # 1=free, 0=wall
        dist_px = cv2.distanceTransform(src, cv2.DIST_L2, 5)
        sigma_px = self.LIDAR_SIGMA / self.MAP_RESOLUTION
        self._likelihood = np.exp(
            -0.5 * (dist_px / sigma_px) ** 2
        ).astype(np.float32)

        self.get_logger().info(
            f'Map loaded: {self._map_w}×{self._map_h} px, '
            f'{len(self._free_pixels)} free pixels, '
            f'likelihood field sigma={self.LIDAR_SIGMA}m')

    # ------------------------------------------------------------------
    # Step D: particle initialisation
    # ------------------------------------------------------------------
    def _init_particles(self) -> np.ndarray:
        """Return (N, 3) array of (x, y, theta) sampled over free space."""
        n = self.N_PARTICLES
        idx = np.random.choice(len(self._free_pixels), size=n, replace=True)

        cols = self._free_pixels[idx, 0].astype(float) + np.random.uniform(-0.5, 0.5, n)
        rows = self._free_pixels[idx, 1].astype(float) + np.random.uniform(-0.5, 0.5, n)

        wx = cols * self.MAP_RESOLUTION + self.MAP_ORIGIN_X
        wy = (self._map_h - 1 - rows) * self.MAP_RESOLUTION + self.MAP_ORIGIN_Y
        wt = np.random.uniform(-math.pi, math.pi, n)

        return np.column_stack([wx, wy, wt])   # (N, 3)

    # ------------------------------------------------------------------
    # Odometry callback — just cache the latest message
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self._last_odom = msg

    # ------------------------------------------------------------------
    # Laser callback — main MCL loop (steps E–I)
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        if self._last_odom is None:
            return

        # ---- Step G: compute dead-reckoning delta from odom ----
        dx_robot, dy_robot, dtheta = 0.0, 0.0, 0.0
        if self._prev_odom is not None:
            dx_robot, dy_robot, dtheta = self._odom_delta(
                self._prev_odom, self._last_odom)

        self._prev_odom = self._last_odom

        # ---- Step H: propagate particles by the estimated displacement ----
        self._particles = self._propagate(self._particles, dx_robot, dy_robot, dtheta)

        # ---- Prepare laser data (step E) ----
        ranges_all = np.asarray(msg.ranges, dtype=np.float32)
        n_beams    = len(ranges_all)
        angles_all = msg.angle_min + np.arange(n_beams) * msg.angle_increment

        # Downsample beams
        beam_idx     = np.arange(0, n_beams, self.BEAM_STRIDE)
        scan_ranges  = ranges_all[beam_idx]
        scan_angles  = angles_all[beam_idx]

        # ---- Step E: score each particle ----
        weights = self._score_particles(self._particles, scan_ranges, scan_angles)

        # ---- Step F: weighted resample ----
        self._particles = self._resample(self._particles, weights)

        # ---- Publish results ----
        self._publish_particles()
        self._publish_best_pose(weights)

    # ------------------------------------------------------------------
    # Step G: odometry delta
    # ------------------------------------------------------------------
    @staticmethod
    def _odom_delta(prev: Odometry, curr: Odometry):
        """
        Compute (dx_robot, dy_robot, dtheta) — the displacement in the
        *robot's previous body frame*.
        """
        # Previous pose in odom frame
        px   = prev.pose.pose.position.x
        py   = prev.pose.pose.position.y
        pqz  = prev.pose.pose.orientation.z
        pqw  = prev.pose.pose.orientation.w
        ptheta = 2.0 * math.atan2(pqz, pqw)

        # Current pose in odom frame
        cx   = curr.pose.pose.position.x
        cy   = curr.pose.pose.position.y
        cqz  = curr.pose.pose.orientation.z
        cqw  = curr.pose.pose.orientation.w
        ctheta = 2.0 * math.atan2(cqz, cqw)

        # World-frame delta
        ddx = cx - px
        ddy = cy - py
        dtheta = ctheta - ptheta
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))  # normalise

        # Rotate delta into the robot's previous frame
        cos_p = math.cos(ptheta)
        sin_p = math.sin(ptheta)
        dx_robot =  ddx * cos_p + ddy * sin_p
        dy_robot = -ddx * sin_p + ddy * cos_p

        return dx_robot, dy_robot, dtheta

    # ------------------------------------------------------------------
    # Step H: propagate particles (dead reckoning + noise)
    # ------------------------------------------------------------------
    def _propagate(self, particles: np.ndarray,
                   dx_robot: float, dy_robot: float, dtheta: float) -> np.ndarray:
        n  = len(particles)
        ct = np.cos(particles[:, 2])
        st = np.sin(particles[:, 2])

        new_p = particles.copy()
        new_p[:, 0] += (dx_robot * ct - dy_robot * st
                        + np.random.normal(0.0, self.MOTION_NOISE_XY, n))
        new_p[:, 1] += (dx_robot * st + dy_robot * ct
                        + np.random.normal(0.0, self.MOTION_NOISE_XY, n))
        new_p[:, 2] += dtheta + np.random.normal(0.0, self.MOTION_NOISE_THETA, n)
        return new_p

    # ------------------------------------------------------------------
    # Step E: vectorised pixel-sum scoring
    # ------------------------------------------------------------------
    def _score_particles(self, particles: np.ndarray,
                         scan_ranges: np.ndarray,
                         scan_angles: np.ndarray) -> np.ndarray:
        """
        Step E: score each particle using a Gaussian likelihood field.

        For each particle, project every laser beam endpoint onto the map
        and look up the pre-computed likelihood value:
          likelihood[pixel] = exp(-0.5 * (dist_to_nearest_wall / sigma)²)

        Beam endpoints AT a wall surface score ≈ 1.0.
        Beam endpoints far inside free space score ≈ 0.0.
        Out-of-bounds endpoints score 0.0 (not 1.0 — avoids rewarding
        particles that are outside the map entirely).

        All operations are vectorised over (N_particles × N_beams).
        """
        px = particles[:, 0]   # (N,)
        py = particles[:, 1]
        pt = particles[:, 2]

        # World angles for every (particle, beam) pair: (N, M)
        world_angles = pt[:, np.newaxis] + scan_angles[np.newaxis, :]

        # Clip invalid / inf ranges
        valid     = np.isfinite(scan_ranges) & (scan_ranges > 0.0)
        r_clipped = np.where(valid, np.minimum(scan_ranges, self.MAX_RANGE),
                             self.MAX_RANGE)  # (M,)

        # Beam endpoint world coords: (N, M)
        ex = px[:, np.newaxis] + r_clipped[np.newaxis, :] * np.cos(world_angles)
        ey = py[:, np.newaxis] + r_clipped[np.newaxis, :] * np.sin(world_angles)

        # ---- Step C: convert to map pixel (col, OpenCV-row) ----
        col    = ((ex - self.MAP_ORIGIN_X) / self.MAP_RESOLUTION).astype(np.int32)
        row_cv = (self._map_h - 1
                  - (ey - self.MAP_ORIGIN_Y) / self.MAP_RESOLUTION).astype(np.int32)

        # Bounds mask — out-of-bounds beams get score 0 (not 1!)
        in_bounds = ((col    >= 0) & (col    < self._map_w) &
                     (row_cv >= 0) & (row_cv < self._map_h))  # (N, M)

        # Clamp indices for safe NumPy lookup (out-of-bounds handled by mask)
        col_c = np.clip(col,    0, self._map_w - 1)
        row_c = np.clip(row_cv, 0, self._map_h - 1)

        # Gaussian likelihood field lookup: (N, M)
        beam_scores = self._likelihood[row_c, col_c]

        # Zero score for out-of-bounds and invalid beams
        beam_scores = np.where(in_bounds, beam_scores, 0.0)
        beam_scores[:, ~valid] = 0.0

        return beam_scores.sum(axis=1)   # (N,)

    # ------------------------------------------------------------------
    # Step F: weighted resampling
    # ------------------------------------------------------------------
    def _resample(self, particles: np.ndarray, weights: np.ndarray) -> np.ndarray:
        """
        Systematic resampling:
          1. Normalise weights.
          2. Draw N_PARTICLES samples proportional to weight.
          3. Add small noise to maintain diversity (prevent filter collapse).

        The noise here is deliberately smaller than NOISE_XY so it does not
        overpower the motion noise applied in _propagate().
        """
        w = np.maximum(weights, 1e-12)
        w = w / w.sum()

        indices     = np.random.choice(len(particles), size=self.N_PARTICLES, p=w)
        new_p       = particles[indices].copy()

        n = self.N_PARTICLES
        new_p[:, 0] += np.random.normal(0.0, self.NOISE_XY,    n)
        new_p[:, 1] += np.random.normal(0.0, self.NOISE_XY,    n)
        new_p[:, 2] += np.random.normal(0.0, self.NOISE_THETA, n)

        return new_p

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_particles(self):
        msg            = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        for p in self._particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = 0.0
            half_yaw = float(p[2]) * 0.5
            pose.orientation.z = math.sin(half_yaw)
            pose.orientation.w = math.cos(half_yaw)
            msg.poses.append(pose)

        self._pub_particles.publish(msg)

    def _publish_best_pose(self, weights: np.ndarray):
        best = int(np.argmax(weights))
        p    = self._particles[best]

        msg            = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = 0.0
        half_yaw = float(p[2]) * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        self._pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
