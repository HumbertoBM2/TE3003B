#!/usr/bin/env python3

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from cv_bridge import CvBridge

MODEL_PATH   = '/home/puzzlebot/best.pt'
CLASS_NAMES  = ['logoe', 'logop', 'logow', 'pallet-detector']
CONF_THRESH  = 0.45
DETECT_HZ    = 5.0   # Hz — Jetson Nano 2GB: no más rápido

# Colores BGR por clase
CLASS_COLORS = {
    'logoe':            (0,   200, 255),   # amarillo
    'logop':            (255, 100,   0),   # azul
    'logow':            (0,   255, 100),   # verde
    'pallet-detector':  (50,   50, 255),   # rojo
}


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path',     MODEL_PATH)
        self.declare_parameter('conf_thresh',    CONF_THRESH)
        self.declare_parameter('detect_hz',      DETECT_HZ)
        self.declare_parameter('imgsz',          320)   # resolución de inferencia — 320 para Jetson Nano

        model_path  = self.get_parameter('model_path').value
        conf_thresh = float(self.get_parameter('conf_thresh').value)
        detect_hz   = float(self.get_parameter('detect_hz').value)
        self._imgsz = int(self.get_parameter('imgsz').value)

        self._bridge   = CvBridge()
        self._model    = None
        self._pending  = None   # último frame sin procesar

        self._load_model(model_path, conf_thresh)

        self.create_subscription(Image, '/video_source/raw',
                                 self._img_cb, qos_profile_sensor_data)

        self.pub_det = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.pub_img = self.create_publisher(CompressedImage, '/yolo/compressed', 10)

        self.create_timer(1.0 / detect_hz, self._detect)

        self.get_logger().info(
            f'yolo_detector listo | modelo={model_path} '
            f'conf={conf_thresh} hz={detect_hz}'
        )

    # ── Carga del modelo ─────────────────────────────────────────────────────

    def _load_model(self, path: str, conf: float):
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self._conf  = conf
            self.get_logger().info(f'Modelo YOLOv8 cargado: {path}')
        except Exception as exc:
            self.get_logger().error(f'No se pudo cargar el modelo YOLO: {exc}')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image):
        self._pending = msg

    def _detect(self):
        if self._pending is None or self._model is None:
            return

        msg = self._pending
        self._pending = None

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}', throttle_duration_sec=5.0)
            return

        try:
            results = self._model(frame, conf=self._conf, imgsz=self._imgsz, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO inference error: {exc}', throttle_duration_sec=5.0)
            return

        det_array = Detection2DArray()
        det_array.header = msg.header
        annotated = frame.copy()

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w  = float(x2 - x1)
                h  = float(y2 - y1)
                cls_id   = int(box.cls[0])
                score    = float(box.conf[0])
                cls_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
                color    = CLASS_COLORS.get(cls_name, (200, 200, 200))

                # Detection2D
                det = Detection2D()
                det.header = msg.header
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score    = score
                det.results.append(hyp)
                det_array.detections.append(det)

                # Anotar imagen
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{cls_name} {score:.2f}'
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(annotated, (x1, y1 - lh - 6), (x1 + lw, y1), color, -1)
                cv2.putText(annotated, label, (x1, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

        self.pub_det.publish(det_array)

        # Publicar imagen anotada como CompressedImage (JPEG) para el dashboard
        try:
            ok, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                cimg = CompressedImage()
                cimg.header = msg.header
                cimg.format = 'jpeg'
                cimg.data   = buf.tobytes()
                self.pub_img.publish(cimg)
        except Exception:
            pass

        if det_array.detections:
            names = [d.results[0].hypothesis.class_id for d in det_array.detections]
            self.get_logger().info(
                f'[yolo] detectado: {names}',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
