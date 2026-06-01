#!/usr/bin/env python3


import base64
import math
import os
import struct
import threading
import time

import rclpy
import yaml as _yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray

try:
    import numpy as np
    import cv2
    _MAP_OK = True
except ImportError:
    _MAP_OK = False

from flask import Flask, Response, jsonify, render_template
from flask import request as freq
from flask_socketio import SocketIO

_latest_frame: bytes = b''
_frame_lock   = threading.Lock()
_frame_event  = threading.Event()

app      = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')




class DashboardBridge(Node):
    def __init__(self):
        super().__init__('web_dashboard')

        qos_be = qos_profile_sensor_data

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

    
        self.create_subscription(Odometry,         '/odom',                    self._odom_cb,       qos_be)
        self.create_subscription(LaserScan,        '/scan',                    self._scan_cb,       qos_be)
        self.create_subscription(CompressedImage,  '/video_source/compressed', self._image_cb,      qos_be)
        self.create_subscription(CompressedImage,  '/yolo/compressed',         self._yolo_img_cb,   qos_be)
        self.create_subscription(Detection2DArray, '/yolo/detections',         self._yolo_det_cb,   qos_be)
        self.create_subscription(String,           '/nav/status',              self._nav_status_cb, qos_rel)
        self.create_subscription(String,           '/slam/status',             self._slam_status_cb,qos_rel)
        self.create_subscription(MarkerArray,      '/slam/map',                self._landmarks_cb,  qos_rel)
        self.create_subscription(OccupancyGrid,    '/map',                     self._occ_map_cb,    qos_latched)
        self.create_subscription(TransformStamped, '/map_to_odom',             self._mo_cb,         qos_rel)
        self.create_subscription(Path,             '/nav/path',                self._nav_path_cb,   qos_rel)

        self.create_subscription(String, '/voice/recognized_command', self._voice_cmd_cb,    qos_rel)
        self.create_subscription(String, '/voice/log_likelihoods',    self._voice_scores_cb, qos_rel)


        self.goal_pub       = self.create_publisher(PoseStamped, '/nav/goal',          qos_rel)
        self.lift_pub       = self.create_publisher(String,      '/lift/command',       qos_rel)
        self.voice_flag_pub = self.create_publisher(Bool,        '/voice/listen_flag',  qos_rel)


        self._last_map_emit = 0.0
        self._mo_x   = 0.0
        self._mo_y   = 0.0
        self._mo_yaw = 0.0

        self.get_logger().info('Dashboard bridge iniciado')

  
    def _mo_cb(self, msg: TransformStamped):
        self._mo_x = msg.transform.translation.x
        self._mo_y = msg.transform.translation.y
        q = msg.transform.rotation
        self._mo_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

 
    def _odom_cb(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        yaw_odom = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        c = math.cos(self._mo_yaw)
        s = math.sin(self._mo_yaw)
        socketio.emit('odom', {
            'x':   round(self._mo_x + c*ox - s*oy, 3),
            'y':   round(self._mo_y + s*ox + c*oy, 3),
            'yaw': round(math.degrees(yaw_odom + self._mo_yaw), 1),
        })

  
    def _scan_cb(self, msg: LaserScan):
        ranges = msg.ranges[::4]
        a_min  = msg.angle_min
        a_inc  = msg.angle_increment * 4
        pts = []
        for i, r in enumerate(ranges):
            if math.isfinite(r) and 0.05 < r < 10.0:
                a = a_min + i * a_inc
                pts.append([round(r*math.cos(a), 3), round(r*math.sin(a), 3)])
        socketio.emit('scan', {'points': pts, 'max_range': msg.range_max})


    def _image_cb(self, msg: CompressedImage):
        global _latest_frame
        with _frame_lock:
            _latest_frame = bytes(msg.data)
        _frame_event.set()

    def _yolo_img_cb(self, msg: CompressedImage):
        socketio.emit('yolo_frame', {'img': base64.b64encode(bytes(msg.data)).decode()})

    def _yolo_det_cb(self, msg: Detection2DArray):
        dets = []
        for d in msg.detections:
            if d.results:
                h = d.results[0].hypothesis
                dets.append({'cls': h.class_id, 'score': round(h.score, 2)})
        socketio.emit('yolo_detections', {'detections': dets})

 
    def _nav_status_cb(self, msg: String):
        socketio.emit('status', {'text': msg.data})

    def _slam_status_cb(self, msg: String):
        socketio.emit('slam_status', {'text': msg.data})

    def _landmarks_cb(self, msg: MarkerArray):
        lms = []
        for m in msg.markers:
            if m.id >= 10000:
                continue
            lms.append({
                'id':       m.id,
                'x':        round(m.pose.position.x, 3),
                'y':        round(m.pose.position.y, 3),
                'detected': bool(m.color.g > 0.5 and m.color.r < 0.5),
            })
        socketio.emit('landmarks', {'data': lms})

    def _occ_map_cb(self, msg: OccupancyGrid):
        if not _MAP_OK:
            return
        now = time.time()
        if now - self._last_map_emit < 2.0:
            return
        self._last_map_emit = now
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        img  = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0] = 235
        img[data > 0]  = 15
        img = cv2.flip(img, 0)
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        socketio.emit('map_grid', {
            'img': base64.b64encode(buf.tobytes()).decode(),
            'ox':  msg.info.origin.position.x,
            'oy':  msg.info.origin.position.y,
            'res': msg.info.resolution,
            'w':   w, 'h': h,
        })


    def _nav_path_cb(self, msg: Path):
        wps = [[round(ps.pose.position.x, 3), round(ps.pose.position.y, 3)]
               for ps in msg.poses]
        socketio.emit('nav_path', {'waypoints': wps})


    def _voice_cmd_cb(self, msg: String):
        socketio.emit('voice_command', {'word': msg.data})

    def _voice_scores_cb(self, msg: String):
        socketio.emit('voice_scores', {'text': msg.data})



@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video')
def video_stream():
    def generate():
        while True:
            _frame_event.wait(timeout=2.0)
            _frame_event.clear()
            with _frame_lock:
                frame = _latest_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/map_image')
def map_image():
    """Sirve ~/maps/current.pgm como JPEG (volteo vertical incluido)."""
    if not _MAP_OK:
        return 'numpy/cv2 not available', 500
    path = os.path.expanduser('~/maps/current.pgm')
    if not os.path.exists(path):
        return 'Map not found', 404
    try:
        with open(path, 'rb') as f:
            magic = f.readline().decode().strip()
            line  = f.readline().decode().strip()
            while line.startswith('#') or not line:
                line = f.readline().decode().strip()
            w, h   = map(int, line.split())
            maxval = int(f.readline().decode().strip())
            raw    = f.read()
            pixels = list(raw) if (magic == 'P5' and maxval < 256) else (
                list(struct.unpack(f'>{w*h}H', raw)) if magic == 'P5'
                else list(map(int, raw.decode().split())))
        arr = np.array(pixels, dtype=np.uint8).reshape(h, w)
        # El PGM ya fue guardado con np.flipud() en amigo_slam_node (fila 0 = norte),
        # así que NO hay que volver a voltear: la imagen ya está lista para pantalla.
        _, buf = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return Response(buf.tobytes(), mimetype='image/jpeg')
    except Exception as e:
        return str(e), 500


@app.route('/map_meta')
def map_meta():
    """Devuelve metadata del mapa guardado: resolution, origin, width, height."""
    yaml_path = os.path.expanduser('~/maps/current.yaml')
    pgm_path  = os.path.expanduser('~/maps/current.pgm')
    if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
        return jsonify({'error': 'map not found'}), 404
    try:
        with open(yaml_path) as f:
            meta = _yaml.safe_load(f)
        with open(pgm_path, 'rb') as f:
            f.readline()
            line = f.readline().decode().strip()
            while line.startswith('#') or not line:
                line = f.readline().decode().strip()
            w, h = map(int, line.split())
        return jsonify({
            'resolution': float(meta['resolution']),
            'origin':     meta['origin'],
            'width':  w,
            'height': h,
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/yolo_push', methods=['POST'])
def yolo_push():
    data = freq.get_json(silent=True) or {}
    if 'img'  in data: socketio.emit('yolo_frame',      {'img':        data['img']})
    if 'dets' in data: socketio.emit('yolo_detections', {'detections': data['dets']})
    return {'ok': True}


# ── SocketIO events ───────────────────────────────────────────────────────────

@socketio.on('send_goal')
def handle_goal(data):
    try:
        msg = PoseStamped()
        msg.header.frame_id    = 'map'
        msg.header.stamp       = bridge_node.get_clock().now().to_msg()
        msg.pose.position.x    = float(data['x'])
        msg.pose.position.y    = float(data['y'])
        msg.pose.orientation.w = 1.0
        bridge_node.goal_pub.publish(msg)
        return {'ok': True}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


@socketio.on('lift')
def handle_lift(data):
    cmd = str(data.get('cmd', 'STOP')).upper()
    if cmd not in ('UP', 'DOWN', 'STOP'):
        cmd = 'STOP'
    msg = String(); msg.data = cmd
    bridge_node.lift_pub.publish(msg)
    return {'ok': True}


@socketio.on('voice_record')
def handle_voice_record(data):
    """Dispara grabación de voz publicando Bool(True) en /voice/listen_flag."""
    msg = Bool(); msg.data = True
    bridge_node.voice_flag_pub.publish(msg)
    return {'ok': True}


# ── Main ──────────────────────────────────────────────────────────────────────

def ros_spin():
    rclpy.spin(bridge_node)


if __name__ == '__main__':
    rclpy.init()
    bridge_node = DashboardBridge()

    threading.Thread(target=ros_spin, daemon=True).start()
    print('\n  Puzzlebot Dashboard: http://localhost:5000\n')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
