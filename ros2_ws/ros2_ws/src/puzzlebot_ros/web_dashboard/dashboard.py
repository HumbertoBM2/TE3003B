#!/usr/bin/env python3
"""
Web Dashboard para Puzzlebot Montacargas
Corre en el PC master en localhost:5000

Instalar dependencias (una vez):
  pip3 install flask flask-socketio eventlet

Correr:
  python3 dashboard.py

Abrir en browser:
  http://localhost:5000
"""

import base64
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

try:
    import numpy as np
    import cv2
    _MAP_ENCODING = True
except ImportError:
    _MAP_ENCODING = False

from flask import Flask, render_template, Response
from flask_socketio import SocketIO

# ── MJPEG frame buffer ────────────────────────────────────────────────────────
_latest_frame: bytes = b''
_frame_lock = threading.Lock()
_frame_event = threading.Event()

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')


# ── ROS2 bridge ──────────────────────────────────────────────────────────────

class DashboardBridge(Node):
    def __init__(self):
        super().__init__('web_dashboard')

        qos_be = qos_profile_sensor_data  # BEST_EFFORT — para LiDAR, odom

        self.create_subscription(Odometry,       '/slam/odom',              self._odom_cb,    qos_be)
        self.create_subscription(LaserScan,      '/scan',                   self._scan_cb,    qos_be)
        self.create_subscription(CompressedImage,'/video_source/compressed',self._image_cb,   qos_be)
        self.create_subscription(String,         '/nav/status',             self._status_cb,  10)
        self.create_subscription(String,         '/slam/status',            self._slam_status_cb, 10)
        self.create_subscription(MarkerArray,    '/slam/map',               self._map_cb,     10)
        self.create_subscription(OccupancyGrid, '/map',                    self._occ_map_cb, 10)

        self.lift_pub = self.create_publisher(String, '/lift/command', 10)
        self._last_map_emit = 0.0

        self.get_logger().info('Web dashboard bridge iniciado')

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        yaw  = math.atan2(siny, cosy)
        data = {
            'x':   round(msg.pose.pose.position.x, 3),
            'y':   round(msg.pose.pose.position.y, 3),
            'yaw': round(math.degrees(yaw), 1),
        }
        socketio.emit('odom', data)

    def _scan_cb(self, msg):
        # Downsample: enviar 1 de cada 4 puntos para no saturar WebSocket
        ranges = msg.ranges[::4]
        angle_min  = msg.angle_min
        angle_incr = msg.angle_increment * 4
        pts = []
        for i, r in enumerate(ranges):
            if math.isfinite(r) and 0.05 < r < 10.0:
                a = angle_min + i * angle_incr
                pts.append([round(r * math.cos(a), 3),
                             round(r * math.sin(a), 3)])
        socketio.emit('scan', {'points': pts, 'max_range': msg.range_max})

    def _image_cb(self, msg):
        global _latest_frame
        with _frame_lock:
            _latest_frame = bytes(msg.data)
        _frame_event.set()

    def _status_cb(self, msg):
        socketio.emit('status', {'text': msg.data})

    def _slam_status_cb(self, msg):
        socketio.emit('slam_status', {'text': msg.data})

    def _map_cb(self, msg):
        landmarks = []
        for m in msg.markers:
            # El SLAM publica CYLINDER (id=aruco_id) + TEXT (id=aruco_id+10000)
            # Solo contar los cilindros para no duplicar el conteo
            if m.id >= 10000:
                continue
            landmarks.append({
                'id': m.id,
                'x':  round(m.pose.position.x, 3),
                'y':  round(m.pose.position.y, 3),
            })
        socketio.emit('landmarks', {'data': landmarks})

    def _occ_map_cb(self, msg):
        if not _MAP_ENCODING:
            return
        now = time.time()
        if now - self._last_map_emit < 2.0:
            return
        self._last_map_emit = now

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        img = np.full((h, w), 100, dtype=np.uint8)  # unknown = mid-gray
        img[data == 0] = 230                          # free = light gray
        img[data > 0] = 15                            # occupied = near-black

        img = cv2.flip(img, 0)  # ROS Y-up → image Y-down

        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        b64 = base64.b64encode(buf.tobytes()).decode()

        socketio.emit('map_grid', {
            'img': b64,
            'ox':  msg.info.origin.position.x,
            'oy':  msg.info.origin.position.y,
            'res': msg.info.resolution,
            'w':   w,
            'h':   h,
        })


# ── Flask routes ─────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video')
def video_stream():
    """MJPEG stream — browser connects once and receives frames as they arrive."""
    def generate():
        while True:
            _frame_event.wait(timeout=2.0)
            _frame_event.clear()
            with _frame_lock:
                frame = _latest_frame
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                )
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on('lift')
def handle_lift(data):
    """Recibe comando de lift ('UP', 'DOWN', 'STOP') y lo publica en ROS."""
    cmd = str(data.get('cmd', 'STOP')).upper()
    if cmd not in ('UP', 'DOWN', 'STOP'):
        cmd = 'STOP'
    msg = String()
    msg.data = cmd
    bridge_node.lift_pub.publish(msg)
    return {'ok': True}


@socketio.on('send_goal')
def handle_goal(data):
    """Recibe goal desde el browser y lo publica en ROS."""
    try:
        x = float(data['x'])
        y = float(data['y'])
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        bridge_node.goal_pub.publish(msg)
        return {'ok': True}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


# ── Main ─────────────────────────────────────────────────────────────────────

def ros_spin():
    rclpy.spin(bridge_node)


if __name__ == '__main__':
    rclpy.init()
    bridge_node = DashboardBridge()
    bridge_node.goal_pub = bridge_node.create_publisher(PoseStamped, '/nav/goal', 10)
    # lift_pub ya está creado en DashboardBridge.__init__

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    print('\n  Puzzlebot Dashboard corriendo en http://localhost:5000\n')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
