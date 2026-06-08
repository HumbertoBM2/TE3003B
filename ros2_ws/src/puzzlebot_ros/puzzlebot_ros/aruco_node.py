#!/usr/bin/env python3

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np

_LEGACY = not hasattr(cv2.aruco, 'ArucoDetector')


def _euler_to_rot(roll, pitch, yaw):
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _make_transform(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = _euler_to_rot(roll, pitch, yaw)
    T[:3,  3] = [x, y, z]
    return T


_T_CAMERA_LINK_OPTICAL = _make_transform(
    0.0, 0.0, 0.0,
    -math.pi / 2.0, 0.0, -math.pi / 2.0,
)


def _rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.ravel()
    return T


def _yaw_from_rot(R):
    return math.atan2(R[1, 0], R[0, 0])


def _rot_to_quat(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s;                 z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return x, y, z, w


def _norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _marker_obj_points(length):
    h = length / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float32)


def _incidence_angle_rad(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    marker_normal  = R[:, 2]
    view_dir       = tvec.ravel()
    view_dir_norm  = view_dir / (np.linalg.norm(view_dir) + 1e-9)
    cos_angle = float(np.clip(np.dot(view_dir_norm, -marker_normal), -1.0, 1.0))
    return math.acos(cos_angle)


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self._declare_parameters()
        self._read_parameters()
        self._load_camera_calibration()
        self._load_extrinsics()
        self._load_marker_map()
        self._setup_detector()
        self._setup_comms()

        self.last_pose       = None
        self._last_pose_time = None
        self._odom_pose_now  = None
        self._odom_pose_det  = None
        self._obj_pts = _marker_obj_points(self.marker_length)
 
        self._solvepnp_flag = cv2.SOLVEPNP_ITERATIVE
        from cv_bridge import CvBridge
        self._bridge = CvBridge()
        self.get_logger().info(
            f'aruco_node listo | OpenCV {cv2.__version__} '
            f'({"legacy" if _LEGACY else "nueva"} API) | '
            f'{len(self.marker_map)} markers conocidos')

    def _declare_parameters(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            cfg_dir = os.path.join(get_package_share_directory('puzzlebot_ros'), 'config')
        except Exception:
            cfg_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')

        def _cfg(name):
            return os.path.join(cfg_dir, name)

        self.declare_parameter('image_topic',           '/video_source/compressed')
        self.declare_parameter('camera_info_file',      _cfg('camera_calibration.yaml'))
        self.declare_parameter('extrinsics_file',       _cfg('camera_extrinsics.yaml'))
        self.declare_parameter('marker_map_file',       _cfg('aruco_map.yaml'))
        self.declare_parameter('dictionary',            'DICT_4X4_50')
        self.declare_parameter('marker_length',         0.10)
        self.declare_parameter('camera_frame',          'camera_link')
        self.declare_parameter('base_frame',            'base_link')
        self.declare_parameter('map_frame',             'map')
        self.declare_parameter('min_marker_area_px',    800.0)
        self.declare_parameter('max_detection_distance', 2.5)
        self.declare_parameter('max_incidence_angle_deg', 75.0)
        self.declare_parameter('max_processing_hz',     8.0)
        self.declare_parameter('max_position_jump',     0.25)
        self.declare_parameter('max_yaw_jump',          0.6)
        self.declare_parameter('reject_unknown_ids',    True)
        self.declare_parameter('last_pose_timeout',     2.0)
        self.declare_parameter('near_marker_position_std', 0.03)
        self.declare_parameter('far_marker_position_std',  0.15)
        self.declare_parameter('near_marker_yaw_std',      0.05)
        self.declare_parameter('far_marker_yaw_std',       0.25)
        self.declare_parameter('map_min_x', 0.0)
        self.declare_parameter('map_max_x', 3.76)
        self.declare_parameter('map_min_y', 0.0)
        self.declare_parameter('map_max_y', 4.86)
        self.declare_parameter('map_bounds_margin', 0.25)
   
        self.declare_parameter('camera_yaw_correction_deg', 0.0)

    def _read_parameters(self):
        g = self.get_parameter
        self.image_topic      = g('image_topic').value
        self.camera_info_file = g('camera_info_file').value
        self.extrinsics_file  = g('extrinsics_file').value
        self.marker_map_file  = g('marker_map_file').value
        self.dict_name        = g('dictionary').value
        self.marker_length    = float(g('marker_length').value)
        self.camera_frame     = g('camera_frame').value
        self.base_frame       = g('base_frame').value
        self.map_frame        = g('map_frame').value
        self.min_area         = float(g('min_marker_area_px').value)
        self.max_dist         = float(g('max_detection_distance').value)
        self.max_pos_jump     = float(g('max_position_jump').value)
        self.max_yaw_jump     = float(g('max_yaw_jump').value)
        self.reject_unknown   = g('reject_unknown_ids').value
        self.near_pos_std     = float(g('near_marker_position_std').value)
        self.far_pos_std      = float(g('far_marker_position_std').value)
        self.near_yaw_std     = float(g('near_marker_yaw_std').value)
        self.far_yaw_std      = float(g('far_marker_yaw_std').value)
        self._last_pose_timeout = float(g('last_pose_timeout').value)
        self._max_incidence_rad = math.radians(float(g('max_incidence_angle_deg').value))
        _max_hz = float(g('max_processing_hz').value)
        self._min_proc_interval = 1.0 / max(_max_hz, 0.1)
        self._last_proc_time    = 0.0
        self._map_min_x = float(g('map_min_x').value)
        self._map_max_x = float(g('map_max_x').value)
        self._map_min_y = float(g('map_min_y').value)
        self._map_max_y = float(g('map_max_y').value)
        self._map_margin = float(g('map_bounds_margin').value)
        self._yaw_corr_rad = math.radians(float(g('camera_yaw_correction_deg').value))

    def _load_camera_calibration(self):
        path = self.camera_info_file
        if not os.path.exists(path):
            self.get_logger().error(f'Calibración no encontrada: {path}')
            raise FileNotFoundError(path)
        with open(path) as f:
            calib = yaml.safe_load(f)
        self.calib_w = calib['image_width']
        self.calib_h = calib['image_height']
        self.K = np.array(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        self.D = np.array(calib['distortion_coefficients']['data'], dtype=np.float64)
        self.get_logger().info(
            f'Calibración: {self.calib_w}x{self.calib_h} '
            f'fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f}')

    def _load_extrinsics(self):
        path = self.extrinsics_file
        if not os.path.exists(path):
            self.get_logger().warn(f'Extrínsecos no encontrados: {path}. Usando identidad.')
            self.T_base_camera = np.eye(4)
        else:
            with open(path) as f:
                data = yaml.safe_load(f)
            ext = data.get('camera_extrinsics', data)
            self.T_base_camera = _make_transform(
                ext['x'], ext['y'], ext['z'],
                ext['roll'], ext['pitch'], ext['yaw'])
        self.T_base_camera_optical = self.T_base_camera @ _T_CAMERA_LINK_OPTICAL

    def _load_marker_map(self):
        path = self.marker_map_file
        if not os.path.exists(path):
            self.get_logger().warn(f'Mapa ArUco no encontrado: {path}')
            self.marker_map = {}
            return
        with open(path) as f:
            data = yaml.safe_load(f)
        self.marker_map = {}
        for mid, pose in data.get('aruco_markers', {}).items():
            self.marker_map[int(mid)] = _make_transform(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw'])
        self.get_logger().info(
            f'Mapa ArUco: {len(self.marker_map)} markers {sorted(self.marker_map.keys())}')

    def _setup_detector(self):
        dict_id    = getattr(cv2.aruco, self.dict_name)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_dict = aruco_dict
        if _LEGACY:
            try:
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            except AttributeError:
                self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = None
        else:
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.aruco_params = None

    def _setup_comms(self):
        self.pub_pose    = self.create_publisher(PoseWithCovarianceStamped, '/aruco/pose', 10)
        self.pub_ids     = self.create_publisher(Int32MultiArray, '/aruco/detected_ids', 10)
     
        self.pub_markers = self.create_publisher(MarkerArray, '/slam/map', 10)

        if 'compressed' in self.image_topic.lower():
            self.create_subscription(
                CompressedImage, self.image_topic, self._cb_compressed,
                qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, self.image_topic, self._cb_raw,
                qos_profile_sensor_data)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._recently_detected: set = set()
        self._last_detected_time = None
        self.create_timer(1.0, self._publish_marker_viz)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_pose_now = (msg.pose.pose.position.x,
                               msg.pose.pose.position.y, yaw)

    def _publish_marker_viz(self):
  
        now = self.get_clock().now().to_msg()
        recently: set = set()
        if self._last_detected_time is not None:
            elapsed = (self.get_clock().now() - self._last_detected_time).nanoseconds * 1e-9
            if elapsed < 3.0:
                recently = self._recently_detected

        ma = MarkerArray()
        for mid, T in self.marker_map.items():
            x = float(T[0, 3])
            y = float(T[1, 3])
            detected = mid in recently

            cyl = Marker()
            cyl.header.stamp    = now
            cyl.header.frame_id = self.map_frame
            cyl.ns = 'aruco_markers'
            cyl.id = mid
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = x
            cyl.pose.position.y = y
            cyl.pose.position.z = 0.10
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.18
            cyl.scale.y = 0.18
            cyl.scale.z = 0.22
            if detected:
                cyl.color.r = 0.1; cyl.color.g = 0.9; cyl.color.b = 0.3
            else:
                cyl.color.r = 0.8; cyl.color.g = 0.6; cyl.color.b = 0.0
            cyl.color.a = 1.0
            cyl.lifetime.sec = 2
            ma.markers.append(cyl)

            txt = Marker()
            txt.header = cyl.header
            txt.ns = 'aruco_markers'
            txt.id = mid + 10000
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.28
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.14
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'ID {mid}'
            txt.lifetime.sec = 2
            ma.markers.append(txt)

        self.pub_markers.publish(ma)

    def _cb_compressed(self, msg: CompressedImage):
        buf   = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            self._process(frame, msg.header.stamp)

    def _cb_raw(self, msg: Image):
        self._process(self._bridge.imgmsg_to_cv2(msg, 'bgr8'), msg.header.stamp)

    def _process(self, frame, stamp):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_proc_time < self._min_proc_interval:
            return
        self._last_proc_time = now_sec

        actual_h, actual_w = frame.shape[:2]
        if actual_w != self.calib_w or actual_h != self.calib_h:
            sx = actual_w / self.calib_w
            sy = actual_h / self.calib_h
            K = self.K.copy()
            K[0, :] *= sx
            K[1, :] *= sy
        else:
            K = self.K

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        if ids is None:
            self.pub_ids.publish(Int32MultiArray(data=[]))
            return

        raw_ids = ids.flatten().tolist()
        self.pub_ids.publish(Int32MultiArray(data=[int(i) for i in raw_ids]))

        candidates = []
        for i, mid in enumerate(ids.flatten().tolist()):
            corner = corners[i]
            area = float(cv2.contourArea(corner.reshape(4, 2)))
            ok, T_cm, dist, rvec, tvec, incidence = self._estimate_pose(corner, K)
            if not ok:
                continue
            result = self._to_robot_pose(mid, T_cm)
            candidates.append(dict(id=mid, area=area, dist=dist,
                                   T_cm=T_cm, rvec=rvec, tvec=tvec,
                                   corner=corner, robot_pose=result,
                                   incidence=incidence))

        valid = self._filter(candidates)
        if not valid:
            return

        x, y, yaw, cov = self._fuse(valid)
        self._publish_pose(x, y, yaw, cov, stamp)
        self.last_pose       = (x, y, yaw)
        self._last_pose_time = self.get_clock().now()
        self._odom_pose_det  = self._odom_pose_now

        self._recently_detected = {c['id'] for c in valid}
        self._last_detected_time = self.get_clock().now()

    def _detect(self, gray):
        if _LEGACY:
            return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        return self.detector.detectMarkers(gray)

    def _estimate_pose(self, corner, K):
        img_pts = corner.reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            _marker_obj_points(self.marker_length), img_pts, K, self.D,
            flags=self._solvepnp_flag)
        if not ok:
            return False, None, 0.0, None, None, 0.0
        dist      = float(np.linalg.norm(tvec))
        incidence = _incidence_angle_rad(rvec, tvec)
        T_cm      = _rvec_tvec_to_T(rvec, tvec)
        return True, T_cm, dist, rvec, tvec, incidence

    def _to_robot_pose(self, marker_id, T_camera_marker):
        if marker_id not in self.marker_map:
            return None
        T_map_marker = self.marker_map[marker_id]
        T_map_cam_opt = T_map_marker @ np.linalg.inv(T_camera_marker)
        T_map_base   = T_map_cam_opt @ np.linalg.inv(self.T_base_camera_optical)
        x   = float(T_map_base[0, 3])
        y   = float(T_map_base[1, 3])
   
        yaw = float(_yaw_from_rot(T_map_base[:3, :3])) + self._yaw_corr_rad
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # normalizar
        marker_yaw = float(_yaw_from_rot(T_map_marker[:3, :3]))
        return (x, y, yaw, marker_yaw)

    def _filter(self, candidates):
        if self.last_pose is not None and self._last_pose_time is not None:
            elapsed = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
            if elapsed > self._last_pose_timeout:
                self.last_pose = None

        valid = []
        for c in candidates:
            mid, area, dist, pose = c['id'], c['area'], c['dist'], c['robot_pose']
            incidence_deg = math.degrees(c.get('incidence', 0.0))
            reason = None
            if self.reject_unknown and mid not in self.marker_map:
                reason = f'ID {mid} desconocido'
            elif area < self.min_area:
                reason = f'ID {mid}: area {area:.0f} < {self.min_area:.0f}'
            elif dist > self.max_dist:
                reason = f'ID {mid}: dist {dist:.2f} > {self.max_dist:.2f}m'
            elif c.get('incidence', 0.0) > self._max_incidence_rad:
                reason = f'ID {mid}: incidencia {incidence_deg:.1f} deg'
            elif pose is None:
                reason = f'ID {mid}: pose global no calculable'
            elif not self._inside_map(pose):
                reason = f'ID {mid}: fuera del mapa ({pose[0]:.2f},{pose[1]:.2f})'
            elif self.last_pose is not None:
                dx  = abs(pose[0] - self.last_pose[0])
                dy  = abs(pose[1] - self.last_pose[1])
                dth = abs(_norm_angle(pose[2] - self.last_pose[2]))
                allowed = self.max_pos_jump
                if self._odom_pose_now and self._odom_pose_det:
                    odx = abs(self._odom_pose_now[0] - self._odom_pose_det[0])
                    ody = abs(self._odom_pose_now[1] - self._odom_pose_det[1])
                    allowed += max(odx, ody)
                if max(dx, dy) > allowed:
                    reason = f'ID {mid}: salto pos ({dx:.2f},{dy:.2f}) > {allowed:.2f}m'
                elif dth > self.max_yaw_jump:
                    reason = f'ID {mid}: salto yaw {math.degrees(dth):.1f} deg'
            if reason:
                self.get_logger().debug(f'Rechazado: {reason}')
            else:
                valid.append(c)
                self.get_logger().info(
                    f'ArUco ID={mid} dist={dist:.2f}m incid={incidence_deg:.1f}deg '
                    f'pose=({pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[2]):.1f}deg)',
                    throttle_duration_sec=0.5)
        return valid

    def _inside_map(self, pose):
        m = max(0.0, self._map_margin)
        return (self._map_min_x - m <= pose[0] <= self._map_max_x + m and
                self._map_min_y - m <= pose[1] <= self._map_max_y + m)

    def _fuse(self, valid):
        weights = [1.0 / max(c['dist'] ** 2, 1e-6) for c in valid]
        W = sum(weights)
        x  = sum(c['robot_pose'][0] * w for c, w in zip(valid, weights)) / W
        y  = sum(c['robot_pose'][1] * w for c, w in zip(valid, weights)) / W
        ss = sum(math.sin(c['robot_pose'][2]) * w for c, w in zip(valid, weights))
        cs = sum(math.cos(c['robot_pose'][2]) * w for c, w in zip(valid, weights))
        yaw = math.atan2(ss / W, cs / W)

        cov_xi, cov_yi, cov_thi = 0.0, 0.0, 0.0
        for c, w in zip(valid, weights):
            cx, cy, cth = self._cov_per_axis(c['dist'], c['incidence'], c['robot_pose'][3])
            cov_xi  += w / max(cx,  1e-9)
            cov_yi  += w / max(cy,  1e-9)
            cov_thi += w / max(cth, 1e-9)

        cov = [0.0] * 36
        cov[0]  = W / max(cov_xi,  1e-9)
        cov[7]  = W / max(cov_yi,  1e-9)
        cov[35] = W / max(cov_thi, 1e-9)
        cov[14] = 0.01; cov[21] = 0.01; cov[28] = 0.01
        return x, y, yaw, cov

    def _cov_per_axis(self, dist, incidence_rad, marker_yaw_in_map):
        NEAR, FAR = 0.3, self.max_dist
        t = float(np.clip((dist - NEAR) / max(FAR - NEAR, 1e-6), 0.0, 1.0))
        std_base = self.near_pos_std + t * (self.far_pos_std - self.near_pos_std)
        std_yaw  = self.near_yaw_std + t * (self.far_yaw_std - self.near_yaw_std)
        cos_inc  = max(math.cos(incidence_rad), 0.125)
        depth_factor = 1.0 / cos_inc
        depth_x = abs(math.cos(marker_yaw_in_map))
        depth_y = abs(math.sin(marker_yaw_in_map))
        fx = depth_x * depth_factor + (1.0 - depth_x)
        fy = depth_y * depth_factor + (1.0 - depth_y)
        fyaw = 1.0 + 0.5 * (depth_factor - 1.0)
        return (std_base * fx) ** 2, (std_base * fy) ** 2, (std_yaw * fyaw) ** 2

    def _publish_pose(self, x, y, yaw, cov, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = _rot_to_quat(_euler_to_rot(0.0, 0.0, yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = cov
        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
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
