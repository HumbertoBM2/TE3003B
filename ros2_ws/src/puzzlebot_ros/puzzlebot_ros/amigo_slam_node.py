#!/usr/bin/env python3


import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped

from .keyframe_manager import KeyframeManager
from .occupancy_grid_map import OccupancyGridMap
from .odometry_buffer import OdometryBuffer
from .amigo_scan_matcher import LocalScanMatcher
from .slam_types import Pose2D


class AmigoSlamNode(Node):
    def __init__(self):
        super().__init__('amigo_slam_node')

       
        self.declare_parameter('map_resolution',   0.05)
        self.declare_parameter('map_width_meters', 4.26)   
        self.declare_parameter('map_height_meters', 5.36)  
        self.declare_parameter('map_origin_x', -0.25)
        self.declare_parameter('map_origin_y', -0.25)

        self.declare_parameter('p_occ',  0.80)
        self.declare_parameter('p_free', 0.45)
        self.declare_parameter('l_clamp', 5.0)
        self.declare_parameter('scan_step', 1)
        self.declare_parameter('max_range_factor', 0.95)
        self.declare_parameter('min_useful_range', 0.20)
        self.declare_parameter('max_mapping_range', 5.5)
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
   
        self.declare_parameter('pose_buffer_sec', 3.0)
        self.declare_parameter('max_scan_pose_age', 0.30)
  
        self.declare_parameter('use_keyframes', True)
        self.declare_parameter('keyframe_min_translation', 0.12)  
        self.declare_parameter('keyframe_min_rotation', math.radians(7.0))  
    
        self.declare_parameter('scan_matching_enabled', True)
        self.declare_parameter('pose_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan_stamped')
        self.declare_parameter('map_frame', 'map')

        res    = float(self.get_parameter('map_resolution').value)
        w_m    = float(self.get_parameter('map_width_meters').value)
        h_m    = float(self.get_parameter('map_height_meters').value)
        width_px  = int(math.ceil(w_m / res))
        height_px = int(math.ceil(h_m / res))

        self._map_frame = self.get_parameter('map_frame').value

        self._grid = OccupancyGridMap(
            size_pixels=max(width_px, height_px),
            size_meters=max(w_m, h_m),
            origin_x=float(self.get_parameter('map_origin_x').value),
            origin_y=float(self.get_parameter('map_origin_y').value),
            p_occ=float(self.get_parameter('p_occ').value),
            p_free=float(self.get_parameter('p_free').value),
            l_clamp=float(self.get_parameter('l_clamp').value),
            scan_step=int(self.get_parameter('scan_step').value),
            max_range_factor=float(self.get_parameter('max_range_factor').value),
            min_useful_range=float(self.get_parameter('min_useful_range').value),
            max_mapping_range=float(self.get_parameter('max_mapping_range').value),
            lidar_x=float(self.get_parameter('lidar_x').value),
            lidar_y=float(self.get_parameter('lidar_y').value),
            lidar_yaw=float(self.get_parameter('lidar_yaw').value),
            width_pixels=width_px,
            height_pixels=height_px,
            resolution=res,
        )
        self._odom_buffer = OdometryBuffer(
            buffer_sec=float(self.get_parameter('pose_buffer_sec').value),
            max_lookup_age=float(self.get_parameter('max_scan_pose_age').value),
        )
        self._keyframes = KeyframeManager(
            enabled=bool(self.get_parameter('use_keyframes').value),
            min_translation=float(self.get_parameter('keyframe_min_translation').value),
            min_rotation=float(self.get_parameter('keyframe_min_rotation').value),
        )
        self._matcher = LocalScanMatcher(
            enabled=bool(self.get_parameter('scan_matching_enabled').value))

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub_map = self.create_publisher(OccupancyGrid, '/map', map_qos)

      
        self._mo_x   = 0.0
        self._mo_y   = 0.0
        self._mo_yaw = 0.0

      
        self._freeze_until = 0.0  

       
        self._v_lin = 0.0
        self._v_ang = 0.0

        self.create_subscription(
            Odometry, self.get_parameter('pose_topic').value,
            self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            TransformStamped, '/map_to_odom', self._map_to_odom_cb, 10)

        self.create_service(Trigger, '/map/save',  self._svc_save)
        self.create_service(Trigger, '/map/clear', self._svc_clear)

        self.create_timer(0.5, self._publish_map)
        self.create_timer(5.0, self._diag)

        self._scans = 0
        self._integrated = 0
        self._skipped = 0
        self._peak_score = 0.0

        self.get_logger().info(
            f'amigo_slam_node listo — {width_px}×{height_px} px @ {res:.3f} m/px, '
            f'pista {w_m:.2f}×{h_m:.2f}m, scan_matching={self._matcher.enabled}')

    def _map_to_odom_cb(self, msg: TransformStamped):
        new_x = msg.transform.translation.x
        new_y = msg.transform.translation.y
        q = msg.transform.rotation
        new_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

     
        import time as _time
        delta = math.hypot(new_x - self._mo_x, new_y - self._mo_y)
        delta_yaw = abs(math.atan2(math.sin(new_yaw - self._mo_yaw),
                                   math.cos(new_yaw - self._mo_yaw)))
        if delta > 0.8 or math.degrees(delta_yaw) > 20.0:
            freeze_t = 3.0   # transición grande → 3s para que blending converja
            self._freeze_until = _time.monotonic() + freeze_t
            self.get_logger().info(
                f'Freeze SLAM {freeze_t:.0f}s — delta={delta:.2f}m/{math.degrees(delta_yaw):.1f}deg')
        elif delta > 0.3 or math.degrees(delta_yaw) > 8.0:
            freeze_t = 1.5   # transición media → 1.5s
            self._freeze_until = max(self._freeze_until, _time.monotonic() + freeze_t)

        self._mo_x   = new_x
        self._mo_y   = new_y
        self._mo_yaw = new_yaw

    def _odom_cb(self, msg: Odometry):
     
        self._v_lin = abs(msg.twist.twist.linear.x)
        self._v_ang = abs(msg.twist.twist.angular.z)
   
        self._odom_buffer.add(msg)

    def _odom_to_map(self, odom_pose: Pose2D) -> Pose2D:
       
        c = math.cos(self._mo_yaw)
        s = math.sin(self._mo_yaw)
        x   = self._mo_x + c * odom_pose.x - s * odom_pose.y
        y   = self._mo_y + s * odom_pose.x + c * odom_pose.y
        yaw = math.atan2(math.sin(odom_pose.yaw + self._mo_yaw),
                         math.cos(odom_pose.yaw + self._mo_yaw))
        return Pose2D(x=x, y=y, yaw=yaw)

    def _scan_cb(self, scan: LaserScan):
        self._scans += 1

    
        import time as _time
        if _time.monotonic() < self._freeze_until:
            return

      
        if self._v_ang > 0.25 and self._v_lin < 0.06:
            return

       
        odom_pose = self._odom_buffer.lookup(scan.header.stamp)
        if odom_pose is None:
            odom_pose = self._odom_buffer.latest_pose
        if odom_pose is None:
            return

      
        pose = self._odom_to_map(odom_pose)

     
        refined = self._matcher.match(scan, pose, self._grid)

       
        corr_d = math.hypot(refined.x - pose.x, refined.y - pose.y)
        corr_y = abs(math.atan2(math.sin(refined.yaw - pose.yaw),
                                math.cos(refined.yaw - pose.yaw)))
        if corr_d > 0.20 or corr_y > math.radians(14.0):
            refined = pose


        _SCORE_QUALITY_FRACTION = 0.45
        _MAP_ESTABLISHED_AT     = 50
        score = self._matcher.last_score
        if score > self._peak_score:
            self._peak_score = score
        if self._integrated >= _MAP_ESTABLISHED_AT and self._peak_score > 50.0:
            threshold = _SCORE_QUALITY_FRACTION * self._peak_score
            if 0.0 < score < threshold:
                self._skipped += 1
                return

        if not self._keyframes.should_integrate(refined):
            return

        if self._grid.integrate_scan(scan, refined):
            self._integrated += 1

    def _publish_map(self):
        self._pub_map.publish(
            self._grid.to_msg(self.get_clock().now().to_msg(), self._map_frame))

    def _diag(self):
        known = int(np.sum(np.abs(self._grid.grid) > 0.5))
        threshold = 0.45 * self._peak_score if self._peak_score > 50.0 else 0.0
        self.get_logger().info(
            f'[amigo_slam] scans={self._scans} integrados={self._integrated} '
            f'saltados={self._skipped} '
            f'celdas={known} '
            f'score={self._matcher.last_score:.0f} '
            f'peak={self._peak_score:.0f} umbral={threshold:.0f} '
            f'buf={"si" if self._odom_buffer.has_pose else "no"}')



    def _svc_clear(self, _req, resp):
        self._grid.clear()
        resp.success = True
        resp.message = 'Mapa reiniciado (todo unknown)'
        self.get_logger().info(resp.message)
        return resp

    def _svc_save(self, _req, resp):
        try:
            maps_dir = os.path.expanduser('~/maps')
            os.makedirs(maps_dir, exist_ok=True)
            ts   = time.strftime('%Y%m%d_%H%M%S')
            base = os.path.join(maps_dir, f'map_{ts}')
            self._write_pgm_yaml(base)
            self._write_pgm_yaml(os.path.join(maps_dir, 'current'))
            resp.success = True
            resp.message = f'Mapa guardado: {base}.pgm y ~/maps/current.pgm'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al guardar: {exc}'
            self.get_logger().error(resp.message)
        return resp

    def _write_pgm_yaml(self, base):
        g = self._grid
        img = np.full((g.height_pixels, g.width_pixels), 205, dtype=np.uint8)
        img[g.grid < -0.5] = 254
        img[g.grid > 0.5] = 0
        img = np.flipud(img)
        h, w = img.shape
        with open(base + '.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(img.tobytes())
        with open(base + '.yaml', 'w') as f:
            f.write(
                f'image: {os.path.basename(base)}.pgm\n'
                f'resolution: {g.resolution}\n'
                f'origin: [{g.origin_x}, {g.origin_y}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def main(args=None):
    rclpy.init(args=args)
    node = AmigoSlamNode()
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
