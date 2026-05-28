#!/usr/bin/env python3
"""
Scan matcher 2D — ICP con mapa local acumulado, anclado al frame del EKF.

El pool de referencia se gestiona en el frame del EKF (no en el propio del
scan_matcher), de modo que las correcciones del EKF (ArUco, etc.) se propagan
automáticamente sin que el ICP acumule su propia deriva.

Publicadores:
  /scan_match/delta  (nav_msgs/Odometry) — movimiento relativo entre dos scans
  /scan_match/pose   (nav_msgs/Odometry) — pose acumulada interna (fallback)
"""

import math
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .my_math import wrap_to_pi, quaternion_from_euler

MAX_ITER         = 30
MAX_DIST         = 0.30    # m — umbral de correspondencia ICP
ICP_PTS          = 400     # puntos del scan actual (downsample)
MAX_REF_PTS      = 3000    # puntos máximos en el pool global
MIN_INLIERS      = 30
MIN_INLIER_RATIO = 0.40
MAX_DX           = 0.40    # m — rechazo de outliers
MAX_DTH          = 0.40    # rad

# Umbral para detectar salto grande en EKF → vaciar pool para evitar ghosting
EKF_JUMP_XY  = 0.25   # m
EKF_JUMP_TH  = 0.12   # rad (~7°)


class ScanMatcher(Node):

    def __init__(self):
        super().__init__('scan_matcher')

        # Pose interna (fallback cuando EKF no está disponible)
        self._x  = 0.0
        self._y  = 0.0
        self._th = 0.0

        # Pose del EKF — fuente principal para el pool
        self._ekf_x   = 0.0
        self._ekf_y   = 0.0
        self._ekf_th  = 0.0
        self._ekf_ok  = False

        # Pool de referencia en frame global (Nx2)
        self._ref_pool: np.ndarray | None = None

        self._first_scan = True
        self._ok   = 0
        self._fail = 0

        qos = qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry, '/slam/odom', self._ekf_cb, qos)

        self.pub_delta = self.create_publisher(Odometry, '/scan_match/delta', 10)
        self.pub_pose  = self.create_publisher(Odometry, '/scan_match/pose',  10)
        self.create_timer(5.0, self._diag)

        self.get_logger().info('scan_matcher (anclado a EKF) iniciado')

    # ── Callback pose EKF ─────────────────────────────────────────────────────

    def _ekf_cb(self, msg: Odometry):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        th = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                        1.0 - 2.0*(q.y*q.y + q.z*q.z))

        if self._ekf_ok:
            dx  = x  - self._ekf_x
            dy  = y  - self._ekf_y
            dth = abs(wrap_to_pi(th - self._ekf_th))
            if math.sqrt(dx*dx + dy*dy) > EKF_JUMP_XY or dth > EKF_JUMP_TH:
                # Corrección grande del EKF → vaciar pool para evitar ghosting
                self._ref_pool = None
                self.get_logger().info(
                    f'EKF salto grande ({math.sqrt(dx*dx+dy*dy):.2f}m, '
                    f'{math.degrees(dth):.1f}°) — pool reiniciado'
                )

        self._ekf_x  = x
        self._ekf_y  = y
        self._ekf_th = th
        self._ekf_ok = True

        # Mantener pose interna sincronizada con el EKF
        self._x  = x
        self._y  = y
        self._th = th

    # ── Pose activa ───────────────────────────────────────────────────────────

    def _pose(self):
        """Devuelve (x, y, th) — EKF si disponible, interna si no."""
        return self._x, self._y, self._th

    # ── Scan → nube de puntos ─────────────────────────────────────────────────

    def _to_pts(self, msg: LaserScan) -> np.ndarray:
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        r = np.asarray(msg.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        return np.column_stack([r[ok] * np.cos(angles[ok]),
                                r[ok] * np.sin(angles[ok])])

    # ── ICP 2D ────────────────────────────────────────────────────────────────

    def _icp(self, cur: np.ndarray, ref: np.ndarray):
        if len(cur) < MIN_INLIERS or len(ref) < MIN_INLIERS:
            return None

        if len(cur) > ICP_PTS:
            step = len(cur) // ICP_PTS
            cur = cur[::step]

        pts  = cur.copy()
        tree = cKDTree(ref)

        R_acc = np.eye(2)
        t_acc = np.zeros(2)

        for _ in range(MAX_ITER):
            dists, idx = tree.query(pts, k=1)
            mask = dists < MAX_DIST
            if mask.sum() < MIN_INLIERS:
                return None

            P, Q = pts[mask], ref[idx[mask]]
            p_c, q_c = P.mean(0), Q.mean(0)
            H = (P - p_c).T @ (Q - q_c)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            t = q_c - R @ p_c

            pts   = (R @ pts.T).T + t
            t_acc = R @ t_acc + t
            R_acc = R @ R_acc

            if np.linalg.norm(t) < 4e-5 and abs(math.atan2(R[1, 0], R[0, 0])) < 4e-5:
                break

        dists_f, _ = tree.query(pts, k=1)
        if (dists_f < MAX_DIST).sum() / len(pts) < MIN_INLIER_RATIO:
            return None

        dx  = float(t_acc[0])
        dy  = float(t_acc[1])
        dth = float(math.atan2(R_acc[1, 0], R_acc[0, 0]))

        if abs(dx) > MAX_DX or abs(dy) > MAX_DX or abs(dth) > MAX_DTH:
            return None

        return dx, dy, dth

    # ── Pool de referencia global ─────────────────────────────────────────────

    def _update_ref_pool(self, pts_robot: np.ndarray):
        """Transforma pts_robot al frame global (usando pose EKF) y añade al pool."""
        x, y, th = self._pose()
        c, s = math.cos(th), math.sin(th)
        gx = x + pts_robot[:, 0] * c - pts_robot[:, 1] * s
        gy = y + pts_robot[:, 0] * s + pts_robot[:, 1] * c
        pts_global = np.column_stack([gx, gy])

        if self._ref_pool is None:
            self._ref_pool = pts_global
        else:
            self._ref_pool = np.vstack([self._ref_pool, pts_global])
            if len(self._ref_pool) > MAX_REF_PTS:
                self._ref_pool = self._ref_pool[-MAX_REF_PTS:]

    def _build_ref_robot(self) -> np.ndarray | None:
        """Proyecta el pool global al frame actual del robot (pose EKF)."""
        if self._ref_pool is None:
            return None
        x, y, th = self._pose()
        ci = math.cos(-th)
        si = math.sin(-th)
        dx = self._ref_pool[:, 0] - x
        dy = self._ref_pool[:, 1] - y
        rx = dx * ci - dy * si
        ry = dx * si + dy * ci
        return np.column_stack([rx, ry])

    # ── Callback principal ────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        pts = self._to_pts(msg)
        if len(pts) < MIN_INLIERS:
            return

        if self._first_scan:
            self._update_ref_pool(pts)
            self._first_scan = False
            return

        ref = self._build_ref_robot()
        if ref is None or len(ref) < MIN_INLIERS:
            self._update_ref_pool(pts)
            return

        result = self._icp(pts, ref)

        if result is None:
            self._fail += 1
            self._update_ref_pool(pts)
            return

        dx, dy, dth = result
        self._ok += 1

        # Si EKF no ha llegado aún, actualizar pose interna con el delta
        if not self._ekf_ok:
            c, s    = math.cos(self._th), math.sin(self._th)
            self._x  += dx * c - dy * s
            self._y  += dx * s + dy * c
            self._th  = wrap_to_pi(self._th + dth)

        # Añadir scan actual al pool (con pose actualizada)
        self._update_ref_pool(pts)

        now = msg.header.stamp

        # Publicar delta
        delta = Odometry()
        delta.header.stamp    = now
        delta.header.frame_id = 'base_link'
        delta.child_frame_id  = 'base_link'
        delta.pose.pose.position.x = dx
        delta.pose.pose.position.y = dy
        delta.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, dth)
        dist = math.sqrt(dx * dx + dy * dy)
        delta.pose.covariance[0]  = max(1e-5, 0.003 * dist)
        delta.pose.covariance[7]  = max(1e-5, 0.003 * dist)
        delta.pose.covariance[35] = max(1e-6, 0.001 * abs(dth))
        self.pub_delta.publish(delta)

        # Publicar pose interna (referencia)
        x, y, th = self._pose()
        pose_msg = Odometry()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'map'
        pose_msg.child_frame_id  = 'base_link'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, th)
        self.pub_pose.publish(pose_msg)

    # ── Diagnóstico ───────────────────────────────────────────────────────────

    def _diag(self):
        ref_size = len(self._ref_pool) if self._ref_pool is not None else 0
        x, y, th = self._pose()
        self.get_logger().info(
            f'[scan_match] ok={self._ok} fail={self._fail} '
            f'ref_pts={ref_size} ekf={"si" if self._ekf_ok else "no"} '
            f'pose=({x:.2f},{y:.2f},{math.degrees(th):.1f}°)'
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
