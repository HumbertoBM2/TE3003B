#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
import cv2


GSTREAMER_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=15/1 ! "
    "nvvidconv ! "
    "video/x-raw,width=640,height=480,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink max-buffers=1 drop=true sync=false"
)

JPEG_QUALITY = 65   
ROTATE_180   = False


TIMER_HZ    = 15
RAW_EVERY   = 1   


COMPRESSED_W = 320
COMPRESSED_H = 240


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.pub_raw = self.create_publisher(Image, '/video_source/raw', qos_be)
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/video_source/compressed', qos_be
        )

        self.cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara con GStreamer. Verifica nvargus-daemon.')
            raise RuntimeError('camera open failed')

        self.get_logger().info(
            f'Camara abierta OK | {COMPRESSED_W}x{COMPRESSED_H}@{TIMER_HZ}Hz '
            f'compressed -> /video_source/compressed | raw -> /video_source/raw'
        )

        self._frame_count = 0
        self._empty_count = 0
        self._reconnect_at = 0.0
        self.timer = self.create_timer(1.0 / TIMER_HZ, self._publish)

    def _publish(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self._empty_count += 1
            now = time.monotonic()
            if self._empty_count >= 10 and now >= self._reconnect_at:
                self.get_logger().error('Pipeline caido — reintentando en 5s')
                self.cap.release()
                self.cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
                self._empty_count = 0
                self._reconnect_at = now + 5.0
            return
        self._empty_count = 0

        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        stamp = self.get_clock().now().to_msg()

 
        small = cv2.resize(frame, (COMPRESSED_W, COMPRESSED_H),
                           interpolation=cv2.INTER_LINEAR)
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub_compressed.publish(msg)

      
        self._frame_count += 1
        if self._frame_count >= RAW_EVERY:
            self._frame_count = 0
            raw = Image()
            raw.header.stamp = stamp
            raw.header.frame_id = 'camera'
            raw.height = frame.shape[0]
            raw.width = frame.shape[1]
            raw.encoding = 'bgr8'
            raw.is_bigendian = 0
            raw.step = raw.width * 3
            raw.data = frame.tobytes()
            self.pub_raw.publish(raw)

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
