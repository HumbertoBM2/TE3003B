#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    _HAS_BRIDGE = True
except ImportError:
    _HAS_BRIDGE = False

try:
    from pyzbar.pyzbar import decode as _pyzbar_decode
    from pyzbar.pyzbar import ZBarSymbol
    _HAS_PYZBAR = True
except ImportError:
    _HAS_PYZBAR = False

_CAM_K = np.array([
    [640.625,   0.0,    327.157],
    [  0.0,   855.883,  223.115],
    [  0.0,     0.0,      1.0  ],
], dtype=np.float64)

# [k1, k2, p1, p2, k3, k4, k5, k6]
_CAM_D = np.array([[
    -10.4637, 87.0480, 0.001223, 0.012005,
    -26.9308, -10.6261, 88.6007, -38.3941,
]], dtype=np.float64)

_QR_PHYS_M = 0.09    
_CAM_PITCH = 0.0    

_QR_OBJ_PTS = np.array([
    [-_QR_PHYS_M / 2,  _QR_PHYS_M / 2, 0.0],   
    [ _QR_PHYS_M / 2,  _QR_PHYS_M / 2, 0.0],  
    [ _QR_PHYS_M / 2, -_QR_PHYS_M / 2, 0.0],   
    [-_QR_PHYS_M / 2, -_QR_PHYS_M / 2, 0.0],   
], dtype=np.float64)  


class QrReaderNode(Node):

    def __init__(self):
        super().__init__('qr_reader_node')

        self.declare_parameter('detect_hz',        10.0)
        self.declare_parameter('raw_timeout_s',    3.0)
        self.declare_parameter('publish_image',    True)
        self.declare_parameter('min_qr_area_px',  200)

        hz          = float(self.get_parameter('detect_hz').value)
        self._pub_img = bool(self.get_parameter('publish_image').value)
        self._min_area = int(self.get_parameter('min_qr_area_px').value)
        raw_timeout = float(self.get_parameter('raw_timeout_s').value)

        self._bridge = CvBridge() if _HAS_BRIDGE else None
        self._frame: np.ndarray | None = None
        self._last_raw_t: float = time.time()
        self._use_compressed = False

        if _HAS_PYZBAR:
            self._qr = None
            self.get_logger().info('QR detector: pyzbar (recomendado para OpenCV 4.2)')
        else:
            try:
                self._qr = cv2.QRCodeDetectorAruco()
                self.get_logger().info('QR detector: QRCodeDetectorAruco (mejorado)')
            except AttributeError:
                self._qr = cv2.QRCodeDetector()
                self.get_logger().info('QR detector: QRCodeDetector (estándar, sin QUIRC)')

        self._pub_decoded = self.create_publisher(String, '/qr/decoded', 10)
        if self._pub_img:
            self._pub_image = self.create_publisher(
                CompressedImage, '/qr/image', qos_profile_sensor_data)

        self._sub_raw = self.create_subscription(
            Image, '/video_source/raw', self._raw_cb, qos_profile_sensor_data)
        self._sub_comp = self.create_subscription(
            CompressedImage, '/video_source/compressed',
            self._comp_cb, qos_profile_sensor_data)

        self.create_timer(1.0 / hz, self._detect)

        self.create_timer(raw_timeout, self._check_raw_timeout)

        self._last_decoded: str = ''
        self._last_decoded_t: float = 0.0
        self._cooldown_s: float = 0.3  

        _p = math.radians(_CAM_PITCH)
        self._R_level = np.array([
            [1.0, 0.0,            0.0          ],
            [0.0, math.cos(_p),  -math.sin(_p) ],
            [0.0, math.sin(_p),   math.cos(_p) ],
        ], dtype=np.float64)
        self._prev_gamma: float | None = None   

        self.get_logger().info(f'qr_reader_node OK — {hz}Hz')

    def _raw_cb(self, msg: Image):
        if self._bridge is not None:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            h, w = msg.height, msg.width
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = buf.reshape((h, w, 3))
        self._frame = frame
        self._last_raw_t = time.time()
        self._use_compressed = False

    def _comp_cb(self, msg: CompressedImage):
        if not self._use_compressed:
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            self._frame = frame

    def _check_raw_timeout(self):
        if not self._use_compressed:
            if time.time() - self._last_raw_t > 3.0:
                self._use_compressed = True
                self.get_logger().warn(
                    'No llega /video_source/raw — usando /video_source/compressed')


    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _gamma_rad(rvec: np.ndarray) -> float:
        R, _ = cv2.Rodrigues(rvec)
        return QrReaderNode._wrap(
            math.atan2(float(R[0, 2]), float(R[2, 2])) + math.pi)

    @staticmethod
    def _sort_corners(pts: np.ndarray) -> np.ndarray:
        pts = pts.reshape(4, 2)
        idx = np.argsort(pts[:, 1])
        top = pts[idx[:2]];  top = top[np.argsort(top[:, 0])]
        bot = pts[idx[2:]];  bot = bot[np.argsort(bot[:, 0])[::-1]]
        return np.vstack([top, bot]).astype(np.float32)

    def _compute_pnp(self, corners: np.ndarray) -> dict | None:
      
        edge = (np.linalg.norm(corners[1] - corners[0])
                + np.linalg.norm(corners[3] - corners[2])) / 2.0
        if edge < 25.0:   # muy pequeño → PnP no confiable
            return None
        corners_d = corners.astype(np.float64).reshape(-1, 1, 2)
        try:
            ok, rvec, tvec = cv2.solvePnP(
                _QR_OBJ_PTS, corners_d, _CAM_K, _CAM_D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                return None
        except Exception as e:
            self.get_logger().warn(
                f'solvePnP error: {e}', throttle_duration_sec=5.0)
            return None

        self._prev_gamma = self._gamma_rad(rvec)

        t_lev = (self._R_level @ tvec.reshape(3)).ravel()
        tx, tz = float(t_lev[0]), float(t_lev[2])

        dist    = math.hypot(tx, tz)
        bearing = math.atan2(tx, tz) 

      
        R, _ = cv2.Rodrigues(rvec)
        nrm  = (self._R_level @ R[:, 2]).astype(float)
        if nrm[0] * (-tx) + nrm[2] * (-tz) < 0.0:
            nrm = -nrm
        nx, nz = nrm[0], nrm[2]
        ln = math.hypot(nx, nz) + 1e-9
        nx /= ln;  nz /= ln

        psi   = math.atan2(-nx, -nz)       
        e_lat = float(-tx * nz + tz * nx)  

        return dict(dist=dist, bearing=bearing, psi=psi, e_lat=e_lat)

    def _detect(self):
        if self._frame is None:
            return

        frame    = self._frame.copy()
        gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated = frame.copy()

        if _HAS_PYZBAR:
            decoded_info = []
            points       = []
            try:
                results = _pyzbar_decode(gray, symbols=[ZBarSymbol.QRCODE])
                for r in results:
                    text = r.data.decode('utf-8', errors='replace')
                    if text:
                        decoded_info.append(text)
                        # pyzbar da el polygon como lista de Point(x,y)
                        pts = np.array([[p.x, p.y] for p in r.polygon],
                                       dtype=np.int32)
                        points.append(pts)
            except Exception as e:
                self.get_logger().warn(
                    f'pyzbar error: {e}', throttle_duration_sec=10.0)
            retval = len(decoded_info) > 0
        else:
            try:
                retval, decoded_info, points, _ = \
                    self._qr.detectAndDecodeMulti(gray)
            except Exception:
                try:
                    data, pts, _ = self._qr.detectAndDecode(gray)
                    retval = bool(data)
                    decoded_info = [data] if data else []
                    points = [pts] if pts is not None else []
                except Exception as e:
                    self.get_logger().warn(
                        f'QR detect error: {e}', throttle_duration_sec=10.0)
                    return

        if retval and decoded_info:
            for i, raw_text in enumerate(decoded_info):
                if not raw_text:
                    continue

                cx_norm = 0.5
                cy_norm = 0.5
                area    = 0.0
                pnp     = None
                if points is not None and i < len(points) and points[i] is not None:
                    pts  = points[i].astype(int)
                    area = cv2.contourArea(pts)
                    if area < self._min_area:
                        continue
                    fh, fw = frame.shape[:2]
                    cx_norm = round(float(pts[:, 0].mean()) / fw, 4)
                    cy_norm = round(float(pts[:, 1].mean()) / fh, 4)
                    cv2.polylines(annotated, [pts], True, (0, 255, 0), 3)
                    cx_px = int(pts[:, 0].mean())
                    cy_px = int(pts[:, 1].mean())
                    cv2.putText(annotated, raw_text[:20], (cx_px - 50, cy_px),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cx_frame = fw // 2
                    cv2.line(annotated, (cx_frame, 0), (cx_frame, fh),
                             (0, 100, 255), 1)
                    if pts.shape[0] == 4:
                        four_pts = pts.astype(np.float32)
                    elif pts.shape[0] >= 3:
                        rect      = cv2.minAreaRect(pts.astype(np.float32))
                        four_pts  = cv2.boxPoints(rect).astype(np.float32)
                    else:
                        four_pts = None
                    if four_pts is not None:
                        sc  = self._sort_corners(four_pts)
                        pnp = self._compute_pnp(sc)
                    if pnp is None:
                        self._prev_gamma = None

                dest = self._parse_dest(raw_text)
                now  = self.get_clock().now().nanoseconds / 1e9

                if (dest == self._last_decoded and
                        now - self._last_decoded_t < self._cooldown_s):
                    continue

                self._last_decoded   = dest
                self._last_decoded_t = now

                payload = json.dumps({
                    'dest':        dest,
                    'raw':         raw_text,
                    'stamp':       now,
                    'cx_norm':     cx_norm,
                    'cy_norm':     cy_norm,
                    'area':        round(area, 1),
                    'bearing_rad': round(pnp['bearing'], 4) if pnp else 0.0,
                    'psi_rad':     round(pnp['psi'],     4) if pnp else 0.0,
                    'e_lat_m':     round(pnp['e_lat'],   4) if pnp else 0.0,
                    'dist_pnp':    round(pnp['dist'],    3) if pnp else 0.0,
                    'pnp_valid':   pnp is not None,
                })
                self._pub_decoded.publish(String(data=payload))
                self.get_logger().info(
                    f'[QR] dest={dest}  cx={cx_norm:.3f}  area={area:.0f}'
                    + (f'  dist={pnp["dist"]:.2f}m bear={math.degrees(pnp["bearing"]):.1f}°'
                       if pnp else '  (no PnP)'))
        else:
            self._prev_gamma = None   
        if self._pub_img:
            ok, buf = cv2.imencode(
                '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                msg_out = CompressedImage()
                msg_out.header.stamp = self.get_clock().now().to_msg()
                msg_out.format = 'jpeg'
                msg_out.data = buf.tobytes()
                self._pub_image.publish(msg_out)

    @staticmethod
    def _parse_dest(raw: str) -> str:
        """Extrae el destino del texto del QR.

        Formatos soportados:
          - JSON:  {"dest": "LOGOE"}  → "LOGOE"
          - plain: "LOGOE"            → "LOGOE"
          - prefixed: "DEST:LOGOE"    → "LOGOE"
        """
        raw = raw.strip()
        try:
            obj = json.loads(raw)
            return str(obj.get('dest', raw)).upper()
        except (json.JSONDecodeError, TypeError):
            pass
        if ':' in raw:
            return raw.split(':', 1)[1].strip().upper()
        return raw.upper()


def main(args=None):
    rclpy.init(args=args)
    node = QrReaderNode()
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
