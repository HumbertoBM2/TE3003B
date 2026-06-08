#!/usr/bin/env python3


from __future__ import annotations

import json
import math
import os
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

try:
    from tf2_ros import Buffer, TransformListener
    _TF2_OK = True
except ImportError:
    _TF2_OK = False

try:
    from .odoo_client import OdooClient
    _ODOO_OK = True
except ImportError:
    _ODOO_OK = False


VOICE_DURATIONS: dict[str, float] = {
    'avanza':    0.8 + 0.4,  
    'retrocede': 0.8 + 0.4,
    'derecha':   0.5 + 0.4,
    'izquierda': 0.5 + 0.4,
    'alto':      0.3,
    'sube':      1.5 + 0.4,
    'baja':      1.5 + 0.4,
    'gira':      1.5 + 0.4,
    'busca':     2.0 + 0.4,
    'empieza':   0.3,       
}
_INTERRUPTIBLE = frozenset()   



LIFT_PRE_SECS    = 1.10   
LIFT_PICK_SECS   = 1.10   
LIFT_DOWN_M1     = 2.21   
LIFT_DOWN_M2     = 1.84
ADVANCE_DIST_M   = 0.22   
ADVANCE_DIST_M2  = 0.09  
M2_DOCK_DIST_M   = 0.03  
ADVANCE_SPEED    = 0.04  
APPROACH_SPEED   = 0.04  
APPROACH_DIST_M  = 0.30  
DIST_TOL_M       = 0.04  
QR_READY_DIST    = 0.40  
ALIGN_K_BEARING  = 1.2   
ALIGN_K_PSI      = 0.5    
ALIGN_K_ELAT     = 0.7  
ALIGN_K_DIST     = 0.5   
ALIGN_V_MAX      = 0.10   
ALIGN_V_REV      = 0.04   
ALIGN_W_MAX      = 0.12   
BEARING_TOL_RAD  = math.radians(5)  
PSI_TOL_RAD      = math.radians(5)   
LAT_TOL_M        = 0.03  
ALIGN_STABLE_N   = 5      
SCAN_SPEED       = 0.08   
QR_TIMEOUT_S     = 180.0 
MIN_LOCK_AREA    = 1500   
APPROACH_STUCK_S = 8.0    
ALIGN_MAX_REV_S  = 5.0    
SCAN_COLLECT_S   = 3.0    
REPOS_BEARING_THRESH = math.radians(30)  
REPOS_EXTRA_M        = 0.20   
REPOS_DRIVE_TOL_M    = 0.12   
REPOS_YAW_TOL_RAD    = math.radians(8)   
REPOS_V              = 0.06 
REPOS_W              = 0.25   
REPOS_TIMEOUT_S      = 20.0   
PSI_REPOS_THRESH     = math.radians(15)  
ALIGN_STUCK_SECS     = 12.0   
DBG_INTERVAL_TICKS   = 20     
ENC_STATIONARY   = 0.20   
NAV_TIMEOUT_S    = 90.0   
TRUCK_DIST_ASSUME = 1.5   

QR_TO_LOGO: dict[str, str] = {
    'logoe':       'logoe',
    'empresa_e':   'logoe',
    'emezon':      'logoe',
    'e':           'logoe',
    'logop':       'logop',
    'empresa_p':   'logop',
    'popsi':       'logop',
    'p':           'logop',
    'logow':       'logow',
    'empresa_w':   'logow',
    'wolmar':      'logow',
    'w':           'logow',
}


DEFAULT_TRUCK_WP: dict[str, list[float]] = {
    'logoe': [3.03, -0.12],   # emezon
    'logow': [2.49, -0.05],   # wolmar
    'logop': [1.97, -0.04],   # popsi
}


class MS(Enum):  
    IDLE          = auto()
    WAIT_WP1      = auto()
    NAV_PICKUP    = auto()
    LIFT_PRE      = auto()   
    ALIGN_QR      = auto()
    LIFT          = auto()  
    ADVANCE       = auto()
    WAIT_WP2      = auto()   
    NAV_DEST      = auto()
    APPROACH_DROP = auto()
    DEPOSIT       = auto()
    RETURN_HOME   = auto()
    TRUCK_MAPPING = auto()
    ABORT         = auto()



_INTERRUPTIBLE = frozenset({
    MS.NAV_PICKUP, MS.NAV_DEST, MS.RETURN_HOME,
    MS.ALIGN_QR, MS.APPROACH_DROP,
    MS.WAIT_WP1, MS.WAIT_WP2,
    MS.TRUCK_MAPPING,
})


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        if _TF2_OK:
            self._tf_buf      = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self)
        else:
            self._tf_buf = None

        self._mode        = 'IDLE'    # IDLE / M1 / M2 / MAPPING
        self._state       = MS.IDLE
        self._nav_status  = 'IDLE'
        self._enter_t     = time.time()

        # Voice interrupt
        self._voice_pause       = False
        self._voice_resume_t    = 0.0
        self._voice_resend_goal = False  

        self._qr_dest: str | None      = None
        self._qr_cx_norm: float        = 0.5
        self._qr_cy_norm: float        = 0.5
        self._qr_area: float           = 0.0
        self._qr_stamp: float          = 0.0
        self._qr_bearing_rad: float    = 0.0
        self._qr_psi_rad: float        = 0.0
        self._qr_e_lat_m: float        = 0.0
        self._qr_dist_pnp: float       = 0.0
        self._qr_pnp_valid: bool       = False
        self._qr_centered_since: float = 0.0   
        self._qr_stable_count: int     = 0      
        self._qr_lock_dest: str | None = None   
        self._qr_last_cx: float        = 0.5    
        self._qr_rev_start: float      = 0.0    
        self._qr_world_x:   float     = 0.0    
        self._qr_world_y:   float     = 0.0   
        self._qr_world_yaw: float     = 0.0    
        self._qr_world_valid: bool    = False  
        self._align_phase:  str       = 'SCAN'  
        self._repos_ap_x:   float     = 0.0   
        self._repos_ap_y:   float     = 0.0  
        self._repos_t0:     float     = 0.0    
        self._track_close_t:  float   = 0.0    
        self._last_stable_t:  float   = 0.0    
        self._scan_candidates:  dict  = {}     
        self._scan_first_qr_t: float  = 0.0  
        self._advance_before_lift: bool = False  
        self._m2_advance_m: float = ADVANCE_DIST_M2  
        self._mission_go: bool = False
        # Odoo
        self._odoo_picking_id: int | None = None
        # Debug
        self._dbg_tick:     int       = 0       
        self._nav_pickup_t: float      = 0.0    
        self._lift_remaining: float    = 0.0    

        self._home_pose: tuple[float, float, float] | None = None  
        self._wp1: tuple[float, float] | None = None
        self._wp2: tuple[float, float] | None = None

        self._truck_map: dict[str, list[float]] = dict(DEFAULT_TRUCK_WP)
        self._truck_detect_buf: dict[str, list[float]] = {}  
        self._last_scan: list[float] = []
        self._last_scan_amin: float  = 0.0
        self._last_scan_ainc: float  = 0.0
        self._enc_r: float = 0.0
        self._enc_l: float = 0.0

        self._pub_goal        = self.create_publisher(PoseStamped, '/nav/goal',             10)
        self._pub_lift        = self.create_publisher(String,      '/lift/command',         10)
        self._pub_vel         = self.create_publisher(Twist,       '/cmd_vel',              10)
        self._pub_state       = self.create_publisher(String,      '/mission/state',        10)
        self._pub_rs          = self.create_publisher(String,      '/mission/robot_state',  10)
        self._pub_truck       = self.create_publisher(String,      '/mission/truck_info',   10)
        self._pub_tmap        = self.create_publisher(String,      '/mission/truck_map',    10)
        self._pub_odoo_status = self.create_publisher(String,      '/mission/odoo_status',  10)

        self.create_subscription(String,    '/mission/mode',     self._mode_cb,  10)
        self.create_subscription(PoseStamped,'/mission/waypoint1',self._wp1_cb,  10)
        self.create_subscription(PoseStamped,'/mission/waypoint2',self._wp2_cb,  10)
        self.create_subscription(Bool,      '/mission/abort',    self._abort_cb, 10)
        self.create_subscription(Bool,      '/mission/go',       self._mission_go_cb, 10)
        self.create_subscription(String,    '/nav/status',       self._nav_cb,   10)
        self.create_subscription(String,    '/qr/decoded',       self._qr_cb,    10)
        self.create_subscription(String,    '/yolo/detections',  self._yolo_cb,  10)
        self.create_subscription(String,    '/voice/recognized_command',
                                            self._voice_cb,      10)
        self.create_subscription(
            LaserScan, '/scan_stamped', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/VelocityEncR', self._encr_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/VelocityEncL', self._encl_cb,
                                 qos_profile_sensor_data)


        _env_candidates = [
            os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env'),
            os.path.expanduser('~/ros2_ws/src/puzzlebot_ros/.env'),
            os.path.join(os.path.dirname(__file__), '.env'),
        ]
        _env_path = next((p for p in _env_candidates if os.path.exists(p)),
                         _env_candidates[0])
        if _ODOO_OK:
            self._odoo = OdooClient.from_env(_env_path)
            if self._odoo.configured:
                self.get_logger().info('Odoo: cliente configurado')
            else:
                self.get_logger().warn('Odoo: .env no encontrado o incompleto — integración desactivada')
        else:
            self._odoo = None
            self.get_logger().warn('Odoo: odoo_client.py no disponible')

        self.create_timer(0.05, self._loop)   # 20 Hz

        self._pub_tmap.publish(String(data=json.dumps(self._truck_map)))
        self.get_logger().info('mission_node E80 listo — IDLE (esperando voz "empieza")')


    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode == self._mode:
            return
        self.get_logger().info(f'[mode] {self._mode} → {new_mode}')
        self._mode = new_mode

        if new_mode == 'IDLE':
            self._transition(MS.ABORT)
        elif new_mode in ('M1', 'M2'):
            if self._state == MS.IDLE:
                self._wp1 = None
                self._wp2 = None
                self._qr_dest = None
                self._transition(MS.WAIT_WP1)
        elif new_mode == 'MAPPING':
            self._transition(MS.TRUCK_MAPPING)

    def _wp1_cb(self, msg: PoseStamped):
        self._wp1 = (msg.pose.position.x, msg.pose.position.y)
        if self._state == MS.WAIT_WP1:
            self.get_logger().info(
                f'WP1 recibido: {self._wp1} — '
                f'{"listo para iniciar" if self._mission_go else "esperando voz empieza"}')

    def _mission_go_cb(self, msg: Bool):
        if msg.data and not self._mission_go:
            self._mission_go = True
            self.get_logger().info('"empieza" recibido — misión autorizada')

    def _wp2_cb(self, msg: PoseStamped):
        self._wp2 = (msg.pose.position.x, msg.pose.position.y)
        if self._state == MS.WAIT_WP2:
            self.get_logger().info(f'WP2 recibido: {self._wp2}')
            self._transition(MS.NAV_DEST)

    def _abort_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('ABORT recibido')
            self._mode = 'IDLE'
            self._transition(MS.ABORT)

    def _nav_cb(self, msg: String):
        self._nav_status = msg.data.strip()

    def _qr_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._qr_dest        = str(d.get('dest', '')).lower().strip()
            self._qr_cx_norm     = float(d.get('cx_norm', 0.5))
            self._qr_cy_norm     = float(d.get('cy_norm', 0.5))
            self._qr_area        = float(d.get('area', 0.0))
            self._qr_stamp       = float(d.get('stamp', time.time()))
            self._qr_bearing_rad = float(d.get('bearing_rad', 0.0))
            self._qr_psi_rad     = float(d.get('psi_rad',     0.0))
            self._qr_e_lat_m     = float(d.get('e_lat_m',     0.0))
            self._qr_dist_pnp    = float(d.get('dist_pnp',    0.0))
            self._qr_pnp_valid   = bool(d.get('pnp_valid',    False))
            if self._qr_dest:
                self._pub_truck.publish(String(data=self._qr_dest))
        except Exception:
            pass

    def _yolo_cb(self, msg: String):
        if self._state != MS.TRUCK_MAPPING:
            return
        robot_moving = (abs(self._enc_r) > ENC_STATIONARY or
                        abs(self._enc_l) > ENC_STATIONARY)
        if robot_moving:
            return
        try:
            data = json.loads(msg.data)
            dets = data.get('dets', [])
            now  = time.time()
            for d in dets:
                cls   = d.get('cls', '')
                score = float(d.get('score', 0))
                if not cls.startswith('logo') or score < 0.70:
                    continue
                if cls not in self._truck_detect_buf:
                    self._truck_detect_buf[cls] = []
                self._truck_detect_buf[cls].append(now)
                self._truck_detect_buf[cls] = [
                    t for t in self._truck_detect_buf[cls] if now - t < 2.0]
                if len(self._truck_detect_buf[cls]) >= 5:
                    self._save_truck_position(cls, d.get('cx_norm', 0.5))
                    self._truck_detect_buf[cls] = []
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan):
        self._last_scan      = list(msg.ranges)
        self._last_scan_amin = msg.angle_min
        self._last_scan_ainc = msg.angle_increment

    def _encr_cb(self, msg: Float32):
        self._enc_r = msg.data

    def _encl_cb(self, msg: Float32):
        self._enc_l = msg.data

    def _voice_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in VOICE_DURATIONS:
            return
        if self._state not in _INTERRUPTIBLE:
            self.get_logger().info(
                f'[VOICE] "{cmd}" ignorado — estado {self._state.name} no interrumpible')
            return
        duration = VOICE_DURATIONS[cmd]
        self._voice_pause    = True
        self._voice_resume_t = time.time() + duration
        self._voice_resend_goal = self._state in {
            MS.NAV_PICKUP, MS.NAV_DEST, MS.RETURN_HOME}
        self._stop()   
        self.get_logger().info(
            f'[VOICE] interrupt "{cmd}" — pausa {duration:.1f}s '
            f'(estado={self._state.name})')


    def _loop(self):
        s   = self._state
        now = time.time()
        ela = now - self._enter_t

        if self._voice_pause:
            if now >= self._voice_resume_t:
                self._voice_pause = False
                self.get_logger().info(
                    f'[VOICE] reanudando misión — estado={s.name}')
                if self._voice_resend_goal:
                    self._resend_current_goal()
                    self._voice_resend_goal = False
            return 

        if s == MS.IDLE:
            pass

        elif s == MS.WAIT_WP1:
            if self._wp1 is not None and self._mission_go:
                self._mission_go = False   
                self._home_pose  = self._get_robot_pose()
                self.get_logger().info(
                    f'WP1={self._wp1} + "empieza" → iniciando NAV_PICKUP')
                self._transition(MS.NAV_PICKUP)

        elif s == MS.NAV_PICKUP:
            if self._mode == 'M1':
                if ela <= LIFT_PRE_SECS:
                    self._pub_lift.publish(String(data='UP'))
                elif ela <= LIFT_PRE_SECS + 0.15:
                    self._pub_lift.publish(String(data='STOP'))
            if self._nav_status == 'ARRIVED':
                self._transition(MS.ALIGN_QR)
            elif self._nav_status.startswith('ERROR') or ela > NAV_TIMEOUT_S:
                self.get_logger().error('Nav a pickup falló')
                self._transition(MS.ABORT)

        elif s == MS.LIFT_PRE:
            if ela < LIFT_PRE_SECS:
                self._pub_lift.publish(String(data='UP'))
            else:
                self._pub_lift.publish(String(data='STOP'))
                self._transition(MS.ALIGN_QR)

        elif s == MS.ALIGN_QR:
            self._do_align_qr(ela)

        elif s == MS.LIFT:
            if ela < LIFT_PICK_SECS:
                self._pub_lift.publish(String(data='UP'))
            else:
                self._pub_lift.publish(String(data='STOP'))
                if self._advance_before_lift:
                    dest_wp = self._resolve_truck_wp()
                    if dest_wp:
                        self._transition(MS.NAV_DEST)
                    else:
                        self.get_logger().error(
                            f'LIFT M2: sin waypoint para truck "{self._qr_dest}"')
                        self._transition(MS.ABORT)
                else:
                    self._transition(MS.ADVANCE)

        elif s == MS.ADVANCE:
            adv_dist     = self._m2_advance_m if self._advance_before_lift else ADVANCE_DIST_M
            advance_secs = adv_dist / ADVANCE_SPEED
            if ela < advance_secs:
                t = Twist(); t.linear.x = ADVANCE_SPEED
                if (self._advance_before_lift and self._qr_pnp_valid
                        and (now - self._qr_stamp) < 0.5):
                    t.angular.z = float(max(-0.06, min(0.06,
                                            -0.8 * self._qr_bearing_rad)))
                self._pub_vel.publish(t)
            else:
                self._stop()
                if self._advance_before_lift:
                    self.get_logger().info(
                        f'ADVANCE M2 {adv_dist*100:.0f}cm completo → LIFT')
                    self._transition(MS.LIFT)
                elif self._mode == 'M1':
                    self._transition(MS.WAIT_WP2)
                else:
                    dest_wp = self._resolve_truck_wp()
                    if dest_wp:
                        self._transition(MS.NAV_DEST)
                    else:
                        self.get_logger().error(
                            f'No hay waypoint para truck "{self._qr_dest}"')
                        self._transition(MS.ABORT)

        elif s == MS.WAIT_WP2:
            pass   

        elif s == MS.NAV_DEST:
            if self._nav_status == 'ARRIVED':
                self._transition(MS.APPROACH_DROP)
            elif self._nav_status.startswith('ERROR') or ela > NAV_TIMEOUT_S:
                self.get_logger().error('Nav a destino falló')
                self._transition(MS.ABORT)

        elif s == MS.APPROACH_DROP:

            dist = self._forward_lidar_dist()
            if dist is None or dist > APPROACH_DIST_M + DIST_TOL_M:
                t = Twist(); t.linear.x = APPROACH_SPEED
                self._pub_vel.publish(t)
            else:
                self._stop()
                self._transition(MS.DEPOSIT)

        elif s == MS.DEPOSIT:
            self._do_deposit(ela)

        elif s == MS.RETURN_HOME:
            if self._nav_status == 'ARRIVED':
                self._transition(MS.IDLE)
            elif self._nav_status.startswith('ERROR') or ela > NAV_TIMEOUT_S:
                self.get_logger().warn('Return home falló — volviendo a IDLE')
                self._transition(MS.IDLE)

        elif s == MS.TRUCK_MAPPING:
            pass  

        elif s == MS.ABORT:
            self._stop()
            self._pub_lift.publish(String(data='STOP'))
            self._transition(MS.IDLE)


    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _update_qr_world(self) -> None:
        """Actualiza la posición mundial estimada del QR usando pose TF2 + PnP."""
        if not self._qr_pnp_valid or self._qr_dist_pnp < 0.05:
            return
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        qr_dir = self._wrap(ryaw - self._qr_bearing_rad)
        self._qr_world_x   = rx + self._qr_dist_pnp * math.cos(qr_dir)
        self._qr_world_y   = ry + self._qr_dist_pnp * math.sin(qr_dir)
        self._qr_world_yaw = qr_dir          # heading ideal del robot al llegar
        self._qr_world_valid = True

    def _compute_approach_point(self) -> None:
        d = QR_READY_DIST + REPOS_EXTRA_M
        self._repos_ap_x = self._qr_world_x - d * math.cos(self._qr_world_yaw)
        self._repos_ap_y = self._qr_world_y - d * math.sin(self._qr_world_yaw)

    def _log_align_dbg(self, elapsed: float, qr_fresh: bool) -> None:
        phase = self._align_phase
        lock  = self._qr_lock_dest or 'none'

        if phase == 'TRACK':
            bear = math.degrees(self._qr_bearing_rad)
            psi  = math.degrees(self._qr_psi_rad)
            self.get_logger().info(
                f'DBG ALIGN_QR phase={phase} lock={lock} '
                f'bear={bear:+.1f}° dist={self._qr_dist_pnp:.3f}m '
                f'psi={psi:+.1f}° elat={self._qr_e_lat_m:+.3f}m '
                f'area={self._qr_area:.0f} stable={self._qr_stable_count}/{ALIGN_STABLE_N} '
                f'pnp={"Y" if self._qr_pnp_valid else "N"} '
                f'world={"({:.2f},{:.2f})".format(self._qr_world_x, self._qr_world_y) if self._qr_world_valid else "?"}'
            )
        elif phase == 'SCAN':
            age_s = time.time() - self._qr_stamp if self._qr_stamp > 0 else 999.0
            self.get_logger().info(
                f'DBG ALIGN_QR phase={phase} lock={lock} '
                f'fresh={qr_fresh} qr_age={age_s:.1f}s area={self._qr_area:.0f} '
                f'last_bear={math.degrees(self._qr_bearing_rad):+.1f}° '
                f'world={"({:.2f},{:.2f})".format(self._qr_world_x, self._qr_world_y) if self._qr_world_valid else "?"} '
                f't={elapsed:.1f}s'
            )
        elif phase == 'REVERSE':
            rev_dur = time.time() - self._qr_rev_start if self._qr_rev_start > 0 else 0.0
            self.get_logger().info(
                f'DBG ALIGN_QR phase={phase} lock={lock} '
                f'rev={rev_dur:.1f}/{ALIGN_MAX_REV_S:.0f}s '
                f'last_bear={math.degrees(self._qr_bearing_rad):+.1f}° '
                f'last_area={self._qr_area:.0f} '
                f'world={"({:.2f},{:.2f})".format(self._qr_world_x, self._qr_world_y) if self._qr_world_valid else "?"}'
            )
        elif phase in ('REPOS_ROT1', 'REPOS_DRIVE', 'REPOS_ROT2'):
            pose = self._get_robot_pose()
            pose_s = f'({pose[0]:.2f},{pose[1]:.2f},{math.degrees(pose[2]):.1f}°)' if pose else '?'
            repos_dur = time.time() - self._repos_t0 if self._repos_t0 > 0 else 0.0
            self.get_logger().info(
                f'DBG ALIGN_QR phase={phase} '
                f'ap=({self._repos_ap_x:.2f},{self._repos_ap_y:.2f}) '
                f'qr_world=({self._qr_world_x:.2f},{self._qr_world_y:.2f}) '
                f'robot={pose_s} fresh={qr_fresh} '
                f'repos_t={repos_dur:.1f}/{REPOS_TIMEOUT_S:.0f}s'
            )

    def _do_align_qr(self, elapsed: float):
        now = time.time()
        t   = Twist()

        if self._lift_remaining > 0.0:
            if elapsed < self._lift_remaining:
                self._pub_lift.publish(String(data='UP'))
                self._pub_vel.publish(t)
                return
            elif elapsed < self._lift_remaining + 0.15:
                self._pub_lift.publish(String(data='STOP'))

        qr_fresh = (now - self._qr_stamp) < 2.0

        if qr_fresh and self._qr_pnp_valid:
            self._update_qr_world()

        self._dbg_tick += 1
        if self._dbg_tick >= DBG_INTERVAL_TICKS:
            self._dbg_tick = 0
            self._log_align_dbg(elapsed, qr_fresh)

        if elapsed > QR_TIMEOUT_S:
            self.get_logger().error(f'ALIGN_QR: timeout {elapsed:.0f}s → ABORT')
            self._transition(MS.ABORT)
            return

        if self._align_phase == 'SCAN':
            if qr_fresh and self._qr_area >= MIN_LOCK_AREA:
                dest = self._qr_dest
                dist = (self._qr_dist_pnp
                        if self._qr_pnp_valid and self._qr_dist_pnp > 0.05
                        else 999.0)

                if dest not in self._scan_candidates or dist < self._scan_candidates[dest]:
                    self._scan_candidates[dest] = dist
                if self._scan_first_qr_t == 0.0:
                    self._scan_first_qr_t = now
                    self.get_logger().info(
                        f'SCAN: primer QR "{dest}" dist={dist:.2f}m — '
                        f'colectando {SCAN_COLLECT_S:.0f}s para elegir el más cercano')


            if (self._scan_first_qr_t > 0
                    and now - self._scan_first_qr_t >= SCAN_COLLECT_S
                    and self._qr_lock_dest is None
                    and self._scan_candidates):
                best = min(self._scan_candidates, key=self._scan_candidates.get)
                self._qr_lock_dest = best
                self.get_logger().info(
                    f'SCAN: lock → dest={best} '
                    f'dist={self._scan_candidates[best]:.2f}m '
                    f'({len(self._scan_candidates)} candidatos: '
                    f'{self._scan_candidates})')

            if (qr_fresh and self._qr_lock_dest is not None
                    and self._qr_dest == self._qr_lock_dest):
                self._align_phase       = 'TRACK'
                self._qr_rev_start      = 0.0
                self._qr_centered_since = 0.0
                self._qr_stable_count   = 0
                _bad_bearing = (abs(self._qr_bearing_rad) > REPOS_BEARING_THRESH)
                _bad_psi     = (self._qr_pnp_valid and
                                abs(self._qr_psi_rad) > PSI_REPOS_THRESH)
                if self._qr_world_valid and (_bad_bearing or _bad_psi):
                    self.get_logger().warn(
                        f'ALIGN_QR: ángulo inicial malo '
                        f'bear={math.degrees(self._qr_bearing_rad):+.1f}° '
                        f'psi={math.degrees(self._qr_psi_rad):+.1f}° → REPOS inmediato')
                    self._compute_approach_point()
                    self._repos_t0    = now
                    self._align_phase = 'REPOS_ROT1'
                self._pub_vel.publish(t)
                return

            t.angular.z = SCAN_SPEED
            self._pub_vel.publish(t)
            return

        if self._align_phase == 'TRACK':
            if not qr_fresh:
                self._align_phase  = 'REVERSE'
                self._qr_rev_start = now
                self._stop()
                self.get_logger().warn(
                    f'ALIGN_QR: QR perdido (lock={self._qr_lock_dest}) → REVERSE')
                return

            if self._qr_lock_dest is not None and self._qr_dest != self._qr_lock_dest:
                t.angular.z = SCAN_SPEED
                self._pub_vel.publish(t)
                self._qr_centered_since = 0.0
                self._qr_stable_count   = 0
                return

            if self._qr_pnp_valid:
                bearing = self._qr_bearing_rad
                psi     = self._qr_psi_rad
                e_lat   = self._qr_e_lat_m
                dist    = self._qr_dist_pnp
            else:
                cx_err   = self._qr_cx_norm - 0.5
                self._qr_last_cx = self._qr_cx_norm
                bearing  = cx_err * math.radians(53)
                psi      = 0.0
                e_lat    = 0.0
                ld       = self._forward_lidar_dist()
                dist     = ld if ld is not None else QR_READY_DIST + 0.15

            e_dist = dist - QR_READY_DIST

            perp = float(max(-0.30, min(0.30,
                            ALIGN_K_PSI * psi + ALIGN_K_ELAT * e_lat)))
            if self._qr_pnp_valid:
                w = float(max(-ALIGN_W_MAX, min(ALIGN_W_MAX,
                              -ALIGN_K_BEARING * bearing - perp)))
            else:
                w = float(max(-0.15, min(0.15, -ALIGN_K_BEARING * bearing)))

            if abs(bearing) < BEARING_TOL_RAD:
                v = 0.0 if abs(e_dist) < DIST_TOL_M else \
                    float(max(-ALIGN_V_REV, min(ALIGN_V_MAX, ALIGN_K_DIST * e_dist)))
            else:
                v = 0.0

            t.angular.z = w
            t.linear.x  = v
            self._pub_vel.publish(t)

            centered = abs(bearing) < BEARING_TOL_RAD
            at_dist  = abs(e_dist)  < DIST_TOL_M
            square   = abs(psi) < PSI_TOL_RAD   or not self._qr_pnp_valid
            on_axis  = abs(e_lat) < LAT_TOL_M   or not self._qr_pnp_valid

            if dist < QR_READY_DIST + 0.15 and self._qr_pnp_valid:
                if self._track_close_t == 0.0:
                    self._track_close_t = now
                    self._last_stable_t = now   
            else:
                self._track_close_t = 0.0

            if centered and at_dist and square and on_axis:
                self._qr_stable_count += 1
                self._last_stable_t = now   
                if self._qr_stable_count >= ALIGN_STABLE_N:
                    self._stop()
                    self.get_logger().info(
                        f'QR alineado — dest={self._qr_dest} '
                        f'dist={dist:.3f}m bear={math.degrees(bearing):.1f}° '
                        f'psi={math.degrees(psi):.1f}°')
                    if self._mode == 'M2':
                        self._advance_before_lift = True
                        self._m2_advance_m = max(
                            ADVANCE_DIST_M2,
                            self._qr_dist_pnp - M2_DOCK_DIST_M)
                        self.get_logger().info(
                            f'ALIGN_QR M2: avanzando {self._m2_advance_m*100:.1f}cm '
                            f'(dist={self._qr_dist_pnp:.3f}m → dock={M2_DOCK_DIST_M*100:.0f}cm) → ADVANCE')
                        if self._odoo and self._odoo.configured and self._qr_dest:
                            def _odoo_done(pid, msg):
                                self._odoo_picking_id = pid
                                self._pub_odoo_status.publish(String(data=msg))
                                self.get_logger().info(f'Odoo: {msg}')
                            self._odoo.create_delivery_async(
                                self._qr_dest, on_done=_odoo_done)
                        self._transition(MS.ADVANCE)
                    else:
                        self._transition(MS.LIFT)
                    return
            else:
                self._qr_stable_count = 0

            if (self._track_close_t > 0 and
                    self._qr_pnp_valid and
                    abs(psi) > PSI_REPOS_THRESH and
                    now - self._last_stable_t > ALIGN_STUCK_SECS and
                    self._qr_world_valid):
                self._stop()
                self.get_logger().warn(
                    f'ALIGN_QR: psi={math.degrees(psi):.1f}° persistente '
                    f'{now - self._last_stable_t:.0f}s sin frame OK → REVERSE+REPOS')
                self._track_close_t = 0.0
                self._align_phase   = 'REVERSE'
                self._qr_rev_start  = now
                return

            if centered:
                if self._qr_centered_since == 0.0:
                    self._qr_centered_since = now
                elif now - self._qr_centered_since > APPROACH_STUCK_S:
                    self._stop()
                    self.get_logger().warn(
                        f'ALIGN_QR: approach atascado '
                        f'{now - self._qr_centered_since:.1f}s '
                        f'dist={dist:.3f}m → forzando avance')
                    if self._mode == 'M2':
                        self._advance_before_lift = True
                        self._m2_advance_m = max(
                            ADVANCE_DIST_M2,
                            self._qr_dist_pnp - M2_DOCK_DIST_M)
                        self._transition(MS.ADVANCE)
                    else:
                        self._transition(MS.LIFT)
            else:
                self._qr_centered_since = 0.0
            return

        if self._align_phase == 'REVERSE':
            if qr_fresh and self._qr_dest == self._qr_lock_dest:
                psi_ok = (not self._qr_pnp_valid or
                          abs(self._qr_psi_rad) < PSI_REPOS_THRESH)
                if psi_ok:
                    self._align_phase  = 'TRACK'
                    self._qr_rev_start = 0.0
                    self._track_close_t = 0.0
                    self._last_stable_t = 0.0
                    self.get_logger().info(
                        f'ALIGN_QR: QR recuperado en REVERSE → TRACK '
                        f'psi={math.degrees(self._qr_psi_rad):.1f}°')
                    return

            rev_dur = now - self._qr_rev_start
            if rev_dur < ALIGN_MAX_REV_S:
                t.linear.x  = -ALIGN_V_REV
                t.angular.z = 0.0
                self._pub_vel.publish(t)
            else:
                self._stop()
                if self._qr_world_valid:
                    self._compute_approach_point()
                    self.get_logger().warn(
                        f'ALIGN_QR: reversa {rev_dur:.1f}s agotada → REPOSICIONAMIENTO '
                        f'ap=({self._repos_ap_x:.2f},{self._repos_ap_y:.2f})')
                    self._repos_t0    = now
                    self._align_phase = 'REPOS_ROT1'
                else:
                    self.get_logger().warn(
                        f'ALIGN_QR: reversa {rev_dur:.1f}s agotada, sin pos. mundial → re-SCAN')
                    self._qr_lock_dest = None
                    self._align_phase  = 'SCAN'
            self._qr_centered_since = 0.0
            self._qr_stable_count   = 0
            return

        _psi_ok_for_track = (not self._qr_pnp_valid or
                             abs(self._qr_psi_rad) < PSI_REPOS_THRESH)
        if qr_fresh and self._qr_dest == self._qr_lock_dest and _psi_ok_for_track:
            prev_phase = self._align_phase
            self._align_phase       = 'TRACK'
            self._qr_rev_start      = 0.0
            self._qr_centered_since = 0.0
            self._qr_stable_count   = 0
            self._track_close_t     = 0.0
            self._last_stable_t     = 0.0
            self.get_logger().info(
                f'ALIGN_QR: QR visible durante {prev_phase} → TRACK '
                f'bear={math.degrees(self._qr_bearing_rad):+.1f}° '
                f'dist={self._qr_dist_pnp:.2f}m psi={math.degrees(self._qr_psi_rad):.1f}°')
            return

        if now - self._repos_t0 > REPOS_TIMEOUT_S:
            self.get_logger().warn(
                f'ALIGN_QR: REPOS timeout {REPOS_TIMEOUT_S:.0f}s → re-SCAN')
            self._qr_lock_dest = None
            self._align_phase  = 'SCAN'
            self._stop()
            return

        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn('ALIGN_QR: sin pose TF2 en REPOS → SCAN')
            self._qr_lock_dest = None
            self._align_phase  = 'SCAN'
            self._stop()
            return

        rx, ry, ryaw = pose

        if self._align_phase == 'REPOS_ROT1':
            dx, dy   = self._repos_ap_x - rx, self._repos_ap_y - ry
            dist_ap  = math.hypot(dx, dy)
            if dist_ap < REPOS_DRIVE_TOL_M:
                self._stop()
                self._align_phase = 'REPOS_ROT2'
                return
            tgt_yaw = math.atan2(dy, dx)
            err_yaw = self._wrap(tgt_yaw - ryaw)
            if abs(err_yaw) < REPOS_YAW_TOL_RAD:
                self._stop()
                self._align_phase = 'REPOS_DRIVE'
                self.get_logger().info(
                    f'ALIGN_QR REPOS_ROT1 completo → DRIVE (dist_ap={dist_ap:.2f}m)')
                return
            t.angular.z = math.copysign(REPOS_W, err_yaw)
            self._pub_vel.publish(t)
            return

        if self._align_phase == 'REPOS_DRIVE':
            dx, dy  = self._repos_ap_x - rx, self._repos_ap_y - ry
            dist_ap = math.hypot(dx, dy)
            if dist_ap < REPOS_DRIVE_TOL_M:
                self._stop()
                self._align_phase = 'REPOS_ROT2'
                self.get_logger().info('ALIGN_QR REPOS_DRIVE completo → ROT2')
                return
            tgt_yaw = math.atan2(dy, dx)
            herr    = self._wrap(tgt_yaw - ryaw)
            t.linear.x  = REPOS_V
            t.angular.z = float(max(-REPOS_W, min(REPOS_W, 2.0 * herr)))
            self._pub_vel.publish(t)
            return

        if self._align_phase == 'REPOS_ROT2':
            err_yaw = self._wrap(self._qr_world_yaw - ryaw)
            if abs(err_yaw) < REPOS_YAW_TOL_RAD:
                self._stop()
                self._qr_lock_dest = None
                self._align_phase  = 'SCAN'
                self.get_logger().info(
                    f'ALIGN_QR REPOS_ROT2 completo → SCAN '
                    f'(apuntando a yaw={math.degrees(self._qr_world_yaw):.1f}°)')
                return
            t.angular.z = math.copysign(REPOS_W, err_yaw)
            self._pub_vel.publish(t)


    def _do_deposit(self, elapsed: float):
        lift_down   = LIFT_DOWN_M1 if self._mode == 'M1' else LIFT_DOWN_M2
        adv_dist    = ADVANCE_DIST_M if self._mode == 'M1' else self._m2_advance_m
        backup_secs = adv_dist / ADVANCE_SPEED
        total_secs  = lift_down + backup_secs

        if elapsed < lift_down:
            self._pub_lift.publish(String(data='DOWN'))
        elif elapsed < total_secs:
            self._pub_lift.publish(String(data='STOP'))
            t = Twist(); t.linear.x = -ADVANCE_SPEED
            self._pub_vel.publish(t)
        else:
            self._stop()
            self._pub_lift.publish(String(data='STOP'))
            height_cm = 4 if self._mode == 'M1' else 2
            self.get_logger().info(
                f'Depósito completo — fork a ~{height_cm}cm del piso')
            if self._odoo and self._odoo_picking_id:
                pid = self._odoo_picking_id
                self._odoo.validate_delivery_async(
                    pid,
                    on_done=lambda ok, msg: (
                        self._pub_odoo_status.publish(String(data=msg)),
                        self.get_logger().info(f'Odoo validate: {msg}')
                    ))
                self._odoo_picking_id = None
            self._transition(MS.RETURN_HOME)


    def _transition(self, new_state: MS):
        old = self._state
        self._state   = new_state
        self._enter_t = time.time()
        self._pub_state.publish(String(data=new_state.name))

        rs_map = {
            MS.IDLE:          'IDLE',
            MS.WAIT_WP1:      'BLOCKED',
            MS.WAIT_WP2:      'BUSY',
            MS.NAV_PICKUP:    'BUSY',
            MS.NAV_DEST:      'BUSY',
            MS.LIFT_PRE:      'LOADING',
            MS.ALIGN_QR:      'LOADING',
            MS.LIFT:          'LOADING',
            MS.ADVANCE:       'LOADING',
            MS.APPROACH_DROP: 'UNLOADING',
            MS.DEPOSIT:       'UNLOADING',
            MS.RETURN_HOME:   'BUSY',
            MS.TRUCK_MAPPING: 'BUSY',
            MS.ABORT:         'IDLE',
        }
        self._pub_rs.publish(String(data=rs_map.get(new_state, 'IDLE')))
        self.get_logger().info(f'[SM] {old.name} → {new_state.name}')

        if new_state == MS.IDLE:
            self._qr_dest            = None
            self._wp1                = None
            self._wp2                = None
            self._mission_go         = False
            self._advance_before_lift = False
            self._m2_advance_m       = ADVANCE_DIST_M2
            self._odoo_picking_id    = None

        elif new_state == MS.WAIT_WP1:
            self.get_logger().info(
                f'Modo {self._mode} — esperando WP1 en el dashboard...')

        elif new_state == MS.NAV_PICKUP:
            self._nav_pickup_t = time.time()
            if self._wp1:
                self._send_goal(*self._wp1)
                if self._mode == 'M1':
                    self.get_logger().info(
                        'NAV_PICKUP M1: subiendo fork 4→7cm durante la navegación')
            else:
                self.get_logger().error('NAV_PICKUP sin WP1')
                self._transition(MS.ABORT)

        elif new_state == MS.LIFT_PRE:
            self.get_logger().info('LIFT_PRE — subiendo fork 4cm→7cm (1.80s)')

        elif new_state == MS.ALIGN_QR:
            self._qr_centered_since  = 0.0
            self._qr_stable_count    = 0
            self._qr_lock_dest       = None
            self._qr_rev_start       = 0.0
            self._qr_world_valid     = False
            self._align_phase        = 'SCAN'
            self._repos_ap_x         = 0.0
            self._repos_ap_y         = 0.0
            self._repos_t0           = 0.0
            self._track_close_t      = 0.0
            self._last_stable_t      = 0.0
            self._scan_candidates    = {}
            self._scan_first_qr_t    = 0.0
            self._advance_before_lift = False
            self._m2_advance_m       = ADVANCE_DIST_M2
            self._dbg_tick           = 0

            if self._mode == 'M1':
                elapsed_nav = time.time() - self._nav_pickup_t
                self._lift_remaining = max(0.0, LIFT_PRE_SECS - elapsed_nav)
            else:
                self._lift_remaining = 0.0
            self.get_logger().info('Buscando QR...')

        elif new_state == MS.WAIT_WP2:
            self.get_logger().info('Esperando WP2 (destino del rack)...')

        elif new_state == MS.NAV_DEST:
            dest_wp = self._resolve_truck_wp()
            if dest_wp:
                self._send_goal(*dest_wp)
            else:
                self.get_logger().error('Sin destino para NAV_DEST')
                self._transition(MS.ABORT)

        elif new_state == MS.RETURN_HOME:
            if self._home_pose:
                self._send_goal(self._home_pose[0], self._home_pose[1],
                                self._home_pose[2])
            else:
                self.get_logger().warn('Sin home guardado — quedando en posición')
                self._transition(MS.IDLE)

        elif new_state == MS.TRUCK_MAPPING:
            self._truck_detect_buf.clear()
            self.get_logger().info(
                'Modo MAPEO — maneja el robot con teleop hacia los camiones')

        elif new_state == MS.ABORT:
            self._stop()
            self._pub_lift.publish(String(data='STOP'))


    def _resolve_truck_wp(self) -> tuple[float, float] | None:
        if self._mode == 'M1' and self._wp2:
            return self._wp2
        if self._mode == 'M2' and self._qr_dest:
            logo = QR_TO_LOGO.get(self._qr_dest, self._qr_dest)
            wp   = self._truck_map.get(logo) or self._truck_map.get(self._qr_dest)
            if wp:
                return (wp[0], wp[1])
        return None

    def _send_goal(self, x: float, y: float, yaw: float = 0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_goal.publish(msg)

    def _stop(self):
        self._pub_vel.publish(Twist())

    def _forward_lidar_dist(self) -> float | None:
        if not self._last_scan:
            return None
        results = []
        for i, r in enumerate(self._last_scan):
            if not math.isfinite(r) or r < 0.05:
                continue
            angle = self._last_scan_amin + i * self._last_scan_ainc
            if abs(angle) < math.radians(15):
                results.append(r)
        return min(results) if results else None

    def _get_robot_pose(self) -> tuple[float, float, float] | None:
        if self._tf_buf is None:
            return None
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, yaw)
        except Exception:
            return None

    def _save_truck_position(self, logo: str, cx_norm: float):

        if logo in DEFAULT_TRUCK_WP:
            self.get_logger().info(
                f'[MAPEO] {logo}: posición real conocida {DEFAULT_TRUCK_WP[logo]} — YOLO ignorado')
            return
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn(
                f'No hay pose para mapear {logo} — usando posición default')
            return
        rx, ry, ryaw = pose
        bearing  = ryaw + (cx_norm - 0.5) * math.radians(60)
        lidar_d  = self._lidar_at_angle(bearing - ryaw)
        dist     = lidar_d if lidar_d is not None else TRUCK_DIST_ASSUME
        tx = round(rx + dist * math.cos(bearing), 3)
        ty = round(ry + dist * math.sin(bearing), 3)
        self._truck_map[logo] = [tx, ty]
        self.get_logger().info(f'[MAPEO] {logo} → ({tx}, {ty})  dist={dist:.2f}m')
        self._pub_tmap.publish(String(data=json.dumps(self._truck_map)))

    def _resend_current_goal(self):
        if self._state == MS.NAV_PICKUP and self._wp1:
            self._send_goal(*self._wp1)
        elif self._state == MS.NAV_DEST:
            wp = self._resolve_truck_wp()
            if wp:
                self._send_goal(*wp)
        elif self._state == MS.RETURN_HOME and self._home_pose:
            self._send_goal(self._home_pose[0], self._home_pose[1],
                            self._home_pose[2])

    def _lidar_at_angle(self, rel_angle: float) -> float | None:
        if not self._last_scan:
            return None
        ang = (rel_angle + math.pi) % (2 * math.pi) - math.pi   # normalizar
        idx = round((ang - self._last_scan_amin) / self._last_scan_ainc)
        idx = max(0, min(idx, len(self._last_scan) - 1))
        r   = self._last_scan[idx]
        return r if math.isfinite(r) and 0.1 < r < 8.0 else None


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
