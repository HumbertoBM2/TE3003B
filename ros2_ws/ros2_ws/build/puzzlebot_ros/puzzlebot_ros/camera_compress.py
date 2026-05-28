#!/usr/bin/env python3
"""
Nodo que suscribe /video_source/raw (Image) y publica
/video_source/compressed (CompressedImage JPEG) para
transmisión eficiente por red WiFi al PC master.

Raw 640x480 rgb8  ~900 KB por frame → JPEG calidad 75 → ~20 KB por frame
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2


class CameraCompress(Node):

    def __init__(self):
        super().__init__('camera_compress')

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, '/video_source/raw', self._callback, qos_sub)
        self.pub = self.create_publisher(
            CompressedImage, '/video_source/compressed', qos_pub)

        self.get_logger().info('camera_compress listo — publicando en /video_source/compressed')

    def _callback(self, msg):
        try:
            channels = len(msg.data) // (msg.height * msg.width)
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)

            enc = msg.encoding.lower()
            if enc in ('rgb8', 'rgb'):
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc in ('bgr8', 'bgr'):
                bgr = arr
            elif enc == 'mono8':
                bgr = arr
            else:
                # Fallback: asumir RGB
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            # La cámara está montada al revés — rotar 180°
            bgr = cv2.rotate(bgr, cv2.ROTATE_180)

            _, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])

            out = CompressedImage()
            out.header = msg.header
            out.header.frame_id = 'camera'   # ros_deep_learning deja frame_id vacío
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'compress error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCompress()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
