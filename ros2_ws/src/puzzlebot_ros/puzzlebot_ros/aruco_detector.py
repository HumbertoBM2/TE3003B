#!/usr/bin/env python3


import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # noqa: F401

import cv2
import cv2.aruco as aruco_mod
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from aruco_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data


def _rvec_to_quat(rvec):
   
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_frame', 'camera')

        self.marker_size = float(self.get_parameter('marker_size').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)

        self.bridge = CvBridge()

       
        self.declare_parameter('fx', 388.44054)
        self.declare_parameter('fy', 518.19597)
        self.declare_parameter('cx', 331.34870)
        self.declare_parameter('cy', 270.91706)
        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        self.cam_matrix = np.array([[fx, 0.0, cx],
                                    [0.0, fy, cy],
                                    [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([-0.338880, 0.116192, 0.000114, -0.001292, 0.0],
                                     dtype=np.float64)

    
        self.aruco_params = aruco_mod.DetectorParameters_create()
        self.aruco_params.minMarkerPerimeterRate = 0.01  # detecta markers pequeños/lejanos
        self._dicts = [
            ('DICT_4X4_50',         aruco_mod.Dictionary_get(aruco_mod.DICT_4X4_50)),
            ('DICT_5X5_50',         aruco_mod.Dictionary_get(aruco_mod.DICT_5X5_50)),
            ('DICT_5X5_250',        aruco_mod.Dictionary_get(aruco_mod.DICT_5X5_250)),
            ('DICT_6X6_50',         aruco_mod.Dictionary_get(aruco_mod.DICT_6X6_50)),
            ('DICT_6X6_250',        aruco_mod.Dictionary_get(aruco_mod.DICT_6X6_250)),
            ('DICT_7X7_50',         aruco_mod.Dictionary_get(aruco_mod.DICT_7X7_50)),
            ('DICT_ARUCO_ORIGINAL', aruco_mod.Dictionary_get(aruco_mod.DICT_ARUCO_ORIGINAL)),
            ('DICT_4X4_1000',       aruco_mod.Dictionary_get(aruco_mod.DICT_4X4_1000)),
        ]
        self._active_dict_name = None  
        self._active_dict = None
        self._frames_checked = 0

        
        self.create_subscription(Image, '/video_source/raw',
                                 self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/video_source/camera_info',
                                 self._caminfo_cb, qos_profile_sensor_data)

        self.pub = self.create_publisher(MarkerArray, '/marker_publisher/markers', 10)

       
        self.create_timer(5.0, self._status_log)

        self.get_logger().info(
            f'aruco_detector OK — probando 8 diccionarios (4x4→7x7), '
            f'marker_size={self.marker_size} m, minMarkerPerimeterRate=0.01'
        )

    def _status_log(self):
        if self._active_dict_name is None:
            self.get_logger().info(
                f'aruco_detector: recibiendo imágenes (frames={self._frames_checked}), '
                f'ningún marcador detectado aún — ¿marcador visible en cámara?'
            )
        else:
            self.get_logger().info(
                f'aruco_detector: activo con {self._active_dict_name}'
            )

    def _caminfo_cb(self, msg):
        self.cam_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _image_cb(self, msg):
        self._frames_checked += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

     
        if self._active_dict is not None:
            corners, ids, _ = aruco_mod.detectMarkers(
                gray, self._active_dict, parameters=self.aruco_params)
        else:
           
            corners, ids = None, None
            for dict_name, aruco_dict in self._dicts:
                c, i, _ = aruco_mod.detectMarkers(
                    gray, aruco_dict, parameters=self.aruco_params)
                if i is not None and len(i) > 0:
                    corners, ids = c, i
                    self._active_dict = aruco_dict
                    self._active_dict_name = dict_name
                    self.get_logger().info(
                        f'aruco_detector: marcador detectado con {dict_name} '
                        f'(ID={i.flatten().tolist()}) — fijando este diccionario'
                    )
                    break

        ma = MarkerArray()
        ma.header.stamp = msg.header.stamp
        ma.header.frame_id = self.camera_frame

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco_mod.estimatePoseSingleMarkers(
                corners, self.marker_size, self.cam_matrix, self.dist_coeffs)

            for mid, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
                qx, qy, qz, qw = _rvec_to_quat(rvec[0])

                m = Marker()
                m.header = ma.header
                m.id = int(mid)
             
                m.pose.pose.position.x = float(tvec[0][0])
                m.pose.pose.position.y = float(tvec[0][1])
                m.pose.pose.position.z = float(tvec[0][2])
                m.pose.pose.orientation.x = qx
                m.pose.pose.orientation.y = qy
                m.pose.pose.orientation.z = qz
                m.pose.pose.orientation.w = qw
                m.confidence = 1.0
                ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
