#!/usr/bin/env python3


import argparse
import base64
import json
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

CLASS_NAMES  = ['logoe', 'logop', 'logow', 'pallet-detector']
CLASS_COLORS = {
    'logoe':            (0,   200, 255),
    'logop':            (255, 100,   0),
    'logow':            (0,   255, 100),
    'pallet-detector':  (50,   50, 255),
}


class YoloMaster(Node):
    def __init__(self, model_path: str, conf: float, hz: float,
                 imgsz: int, dashboard_url: str):
        super().__init__('yolo_master')

        self._conf          = conf
        self._imgsz         = imgsz
        self._dashboard_url = dashboard_url.rstrip('/') + '/yolo_push'
        self._frame         = None
        self._model         = None

        self._load_model(model_path)

        self.create_subscription(
            CompressedImage, '/video_source/compressed',
            self._img_cb, qos_profile_sensor_data)

        self.create_timer(1.0 / hz, self._detect)

        self.get_logger().info(
            f'yolo_master | modelo={model_path} conf={conf} '
            f'hz={hz} imgsz={imgsz} → {self._dashboard_url}'
        )

    def _load_model(self, path: str):
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self.get_logger().info(f'Modelo YOLO cargado: {path}')
        except Exception as exc:
            self.get_logger().error(f'No se pudo cargar YOLO: {exc}')

    def _img_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            self._frame = frame

    def _detect(self):
        if self._frame is None or self._model is None:
            return

        frame = self._frame
        self._frame = None

        try:
            results = self._model(frame, conf=self._conf,
                                  imgsz=self._imgsz, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO error: {exc}', throttle_duration_sec=5.0)
            return

        annotated = frame.copy()
        dets      = []

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                cls_id   = int(box.cls[0])
                score    = float(box.conf[0])
                cls_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
                color    = CLASS_COLORS.get(cls_name, (200, 200, 200))

                dets.append({'cls': cls_name, 'score': round(score, 2)})

                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{cls_name} {score:.2f}'
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(annotated, (x1, y1 - lh - 8), (x1 + lw + 4, y1), color, -1)
                cv2.putText(annotated, label, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


        ok, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            b64 = base64.b64encode(buf.tobytes()).decode()
            self._push(b64, dets)

        if dets:
            names = [d['cls'] for d in dets]
            self.get_logger().info(f'[yolo] {names}', throttle_duration_sec=1.0)

    def _push(self, img_b64: str, dets: list):
        """HTTP POST directo al dashboard — no pasa por ROS2."""
        try:
            payload = json.dumps({'img': img_b64, 'dets': dets}).encode()
            req = urllib.request.Request(
                self._dashboard_url,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            urllib.request.urlopen(req, timeout=0.3)
        except Exception:
            pass   # dashboard no disponible — silencioso


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',     default='best.pt')
    parser.add_argument('--conf',      type=float, default=0.45)
    parser.add_argument('--hz',        type=float, default=5.0)
    parser.add_argument('--imgsz',     type=int,   default=640)
    parser.add_argument('--dashboard', default='http://localhost:5000')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = YoloMaster(args.model, args.conf, args.hz, args.imgsz, args.dashboard)
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
