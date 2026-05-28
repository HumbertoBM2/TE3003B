"""
EKF-SLAM desde cero para Puzzlebot (montacargas autónomo).
Implementado sin librerías de SLAM prefabricadas.

Algoritmo: Extended Kalman Filter SLAM
- Estado:  mu = [x, y, theta, x_L1, y_L1, x_L2, y_L2, ..., x_Ln, y_Ln]^T
- Covarianza: Sigma  (3+2n x 3+2n)
- Predicción: modelo cinemático diferencial con encoders de rueda
- Corrección: observaciones de marcadores ArUco (distancia y ángulo)
- Inicialización de landmarks: primer avistamiento de un ArUco ID agrega al estado

Tópicos suscritos:
  /VelocityEncR  (std_msgs/Float32) — velocidad angular rueda derecha [rad/s]
  /VelocityEncL  (std_msgs/Float32) — velocidad angular rueda izquierda [rad/s]
  /marker_publisher/markers  (aruco_msgs/MarkerArray) — detecciones ArUco

Tópicos publicados:
  /slam/odom     (nav_msgs/Odometry) — pose estimada del robot
  /slam/map      (visualization_msgs/MarkerArray) — landmarks en el mapa
"""

import json
import os

import rclpy
from rclpy.node import Node
from rclpy import qos

import math
import numpy as np
from timeit import default_timer as timer
from typing import Dict

from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray as VizMarkerArray
from aruco_msgs.msg import MarkerArray

from tf2_ros import TransformBroadcaster

from .my_math import wrap_to_pi, quaternion_from_euler

LANDMARKS_FILE = os.path.expanduser('~/maps/landmarks.json')


# ─────────────────────────────────────────────────────────────────────────────
# Parámetros del robot
# ─────────────────────────────────────────────────────────────────────────────
WHEEL_RADIUS   = 0.05       # metros
ROBOT_WIDTH    = 0.18       # metros (distancia entre ruedas — medido)

# Ruido de proceso: varianza por unidad de velocidad (modelo de error en ruedas)
SIGMA_ENC      = 0.1

# Ruido de observación ArUco
R_RANGE        = 0.05       # varianza en distancia [m²]
R_BEARING      = 0.02       # varianza en ángulo   [rad²]

# ── Mapa físico de ArUcos — pista 3.76m (X) × 4.86m (Y) ─────────────────────
# Origen (0,0) = esquina inferior izquierda, X→ derecha, Y↑ arriba
# El robot debe arrancar en (0,0) mirando al este (+X) para que los frames coincidan.
KNOWN_ARUCO_MAP: dict[int, tuple[float, float]] = {
    0: (0.00, 3.90),   # pared oeste, arriba
    1: (1.88, 4.86),   # pared norte, centro
    2: (3.76, 3.90),   # pared este, arriba
    3: (3.76, 1.04),   # pared este, abajo
    4: (0.00, 1.04),   # pared oeste, abajo
}
KNOWN_LM_COV   = 1e-6   # covarianza inicial para posición conocida (prácticamente fija)
KNOWN_R_FACTOR = 0.08   # reducción del ruido de observación para marcadores conocidos

# Distancia mínima para considerar que llegamos a un waypoint
DMIN_WP        = 0.08       # metros


class EkfSlam(Node):
    """Nodo ROS 2 que corre EKF-SLAM en tiempo real."""

    def __init__(self):
        super().__init__('ekf_slam')

        # ── Parámetros calibrables sin tocar código ─────────────────────────
        self.wheel_radius = float(self.declare_parameter('wheel_radius', WHEEL_RADIUS).value)
        self.robot_width = float(self.declare_parameter('robot_width', ROBOT_WIDTH).value)
        self.sigma_enc = float(self.declare_parameter('sigma_enc', SIGMA_ENC).value)
        self.r_range = float(self.declare_parameter('r_range', R_RANGE).value)
        self.r_bearing = float(self.declare_parameter('r_bearing', R_BEARING).value)
        self.landmark_id_min = int(self.declare_parameter('landmark_id_min', 0).value)
        self.landmark_id_max = int(self.declare_parameter('landmark_id_max', 9).value)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_odom = self.create_publisher(Odometry, '/slam/odom', 10)
        self.pub_map  = self.create_publisher(VizMarkerArray, '/slam/map', 10)
        self.pub_status = self.create_publisher(String, '/slam/status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ─────────────────────────────────────────────────────
        qos_s = qos.qos_profile_sensor_data
        self.create_subscription(Float32, 'VelocityEncR', self._enc_r_cb, qos_s)
        self.create_subscription(Float32, 'VelocityEncL', self._enc_l_cb, qos_s)
        self.create_subscription(MarkerArray, '/marker_publisher/markers',
                                 self._aruco_cb, qos_s)
        self.create_subscription(Odometry, '/scan_match/delta',
                                 self._scan_match_cb, qos_s)

        # ── Servicios ────────────────────────────────────────────────────────
        self.create_service(Trigger, '/slam/save_landmarks', self._svc_save_landmarks)
        self.create_service(Trigger, '/slam/clear_landmarks', self._svc_clear_landmarks)

        # ── Estado EKF-SLAM ─────────────────────────────────────────────────
        # mu[0:3] = [x, y, theta] del robot
        # mu[3+2k : 3+2k+2] = [x_Lk, y_Lk] del k-ésimo landmark
        self.mu    = np.zeros(3)             # vector de estado (crece con landmarks)
        self.Sigma = np.eye(3) * 1e-6        # covarianza inicial muy pequeña (conocemos pose inicial)

        # Diccionario: aruco_id → índice k en landmarks (0-based)
        self.landmark_id_to_idx: Dict[int, int] = {}

        # ── Velocidades de encoders ──────────────────────────────────────────
        self.vel_r = 0.0   # rad/s rueda derecha
        self.vel_l = 0.0   # rad/s rueda izquierda

        # ── Scan matching — referencia anclada al EKF ────────────────────────
        self._sm_ref_x  = 0.0
        self._sm_ref_y  = 0.0
        self._sm_ref_th = 0.0
        self._sm_first  = True   # inicializar referencia con el primer delta

        # ── Diagnóstico para dashboard/calibración ──────────────────────────
        self.last_aruco_count = 0
        self.last_v = 0.0
        self.last_w = 0.0

        # ── Timer de predicción ──────────────────────────────────────────────
        self.dt = 0.02     # 50 Hz
        self._last_t = timer()
        self.create_timer(self.dt, self._predict_loop)
        self.create_timer(1.0, self._publish_status)

        # ── Cargar landmarks guardados si existen ────────────────────────────
        self._load_landmarks()

        self.get_logger().info(
            'EKF-SLAM iniciado. Params: '
            f'wheel_radius={self.wheel_radius:.4f}, robot_width={self.robot_width:.4f}, '
            f'sigma_enc={self.sigma_enc:.4f}, aruco_ids={self.landmark_id_min}-{self.landmark_id_max}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks de encoders
    # ─────────────────────────────────────────────────────────────────────────

    def _enc_r_cb(self, msg: Float32):
        self.vel_r = msg.data

    def _enc_l_cb(self, msg: Float32):
        self.vel_l = msg.data

    def _scan_match_cb(self, msg: Odometry):
        """Delta de scan matching → corrección EKF con referencia anclada al EKF.

        La referencia (_sm_ref_*) es la pose EKF después de la última corrección.
        Así el scan match opera en el mismo frame que el EKF y no hay deriva de
        marco de referencia aunque el scan_matcher se reinicie.
        """
        dx  = msg.pose.pose.position.x
        dy  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        dth = math.atan2(siny, cosy)

        # Primer delta: inicializar referencia con la pose EKF actual
        if self._sm_first:
            self._sm_ref_x  = self.mu[0]
            self._sm_ref_y  = self.mu[1]
            self._sm_ref_th = self.mu[2]
            self._sm_first  = False
            return

        # Pose prevista desde la referencia + delta del ICP
        th_ref = self._sm_ref_th
        c, s   = math.cos(th_ref), math.sin(th_ref)
        x_pred = self._sm_ref_x + dx * c - dy * s
        y_pred = self._sm_ref_y + dx * s + dy * c
        th_pred = wrap_to_pi(th_ref + dth)

        # EKF measurement update: pose ICP como medición de la pose del robot
        n = len(self.mu)
        inn = np.array([
            x_pred  - self.mu[0],
            y_pred  - self.mu[1],
            wrap_to_pi(th_pred - self.mu[2]),
        ])

        H = np.zeros((3, n))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        dist = math.sqrt(dx * dx + dy * dy)
        R_sm = np.diag([
            max(0.002, 0.015 * dist),
            max(0.002, 0.015 * dist),
            max(0.0005, 0.006 * abs(dth)),
        ])

        S = H @ self.Sigma @ H.T + R_sm
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        self.mu    = self.mu + K @ inn
        self.mu[2] = wrap_to_pi(self.mu[2])
        self.Sigma = (np.eye(n) - K @ H) @ self.Sigma

        # Actualizar referencia a la pose EKF ya corregida
        self._sm_ref_x  = self.mu[0]
        self._sm_ref_y  = self.mu[1]
        self._sm_ref_th = self.mu[2]

    # ─────────────────────────────────────────────────────────────────────────
    # Callback ArUco — paso de corrección EKF
    # ─────────────────────────────────────────────────────────────────────────

    def _aruco_cb(self, msg: MarkerArray):
        self.last_aruco_count = len(msg.markers)
        for aruco in msg.markers:
            # Coordenadas del marcador en el frame de la cámara
            # aruco_ros publica: z = profundidad (frente), x = lateral
            dx =  aruco.pose.pose.position.z   # frente del robot
            dy = -aruco.pose.pose.position.x   # lateral izquierda positivo

            obs_range   = math.sqrt(dx**2 + dy**2)
            obs_bearing = math.atan2(dy, dx)   # ángulo relativo al robot

            if obs_range < 0.05:
                continue   # lectura inválida

            aruco_id = aruco.id
            if aruco_id < self.landmark_id_min or aruco_id > self.landmark_id_max:
                continue

            # ── Inicializar landmark si es nuevo ─────────────────────────
            if aruco_id not in self.landmark_id_to_idx:
                self._add_landmark(aruco_id, obs_range, obs_bearing)

            # ── Corrección EKF ────────────────────────────────────────────
            self._ekf_update(aruco_id, obs_range, obs_bearing)

    # ─────────────────────────────────────────────────────────────────────────
    # Timer de predicción — paso de predicción EKF (cinemática diferencial)
    # ─────────────────────────────────────────────────────────────────────────

    def _predict_loop(self):
        now = timer()
        dt  = now - self._last_t
        self._last_t = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Predicción continua con encoders (scan match llega como corrección separada)
        self._ekf_predict(dt)

        self._publish_state()

    # ─────────────────────────────────────────────────────────────────────────
    # EKF: Predicción
    # ─────────────────────────────────────────────────────────────────────────

    def _ekf_predict(self, dt: float):
        R  = self.wheel_radius
        L  = self.robot_width

        # Velocidades lineales de cada rueda
        vr = self.vel_r * R
        vl = self.vel_l * R

        # Velocidad lineal y angular del robot
        v  = (vr + vl) / 2.0
        w  = (vr - vl) / L
        self.last_v = v
        self.last_w = w

        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        n = len(self.mu)   # tamaño total del estado

        # ── Nuevo estado predicho ────────────────────────────────────────
        th_new = wrap_to_pi(th + w * dt)
        x_new  = x + v * dt * math.cos(th + w * dt / 2.0)
        y_new  = y + v * dt * math.sin(th + w * dt / 2.0)

        self.mu[0] = x_new
        self.mu[1] = y_new
        self.mu[2] = th_new

        # ── Jacobiano del modelo de movimiento respecto al estado ────────
        # Solo afecta la parte del robot (3x3); landmarks no se mueven
        F_x = np.eye(n)
        F_x[0, 2] = -v * dt * math.sin(th + w * dt / 2.0)
        F_x[1, 2] =  v * dt * math.cos(th + w * dt / 2.0)

        # ── Jacobiano respecto al ruido de ruedas (3x2) ──────────────────
        dH = np.zeros((n, 2))
        dH[0, 0] =  0.5 * dt * R * math.cos(th)
        dH[0, 1] =  0.5 * dt * R * math.cos(th)
        dH[1, 0] =  0.5 * dt * R * math.sin(th)
        dH[1, 1] =  0.5 * dt * R * math.sin(th)
        dH[2, 0] =  dt * R / L
        dH[2, 1] = -dt * R / L

        # ── Covarianza del ruido de proceso ──────────────────────────────
        K_noise = np.array([
            [self.sigma_enc * abs(self.vel_r), 0.0],
            [0.0, self.sigma_enc * abs(self.vel_l)]
        ])

        Q = dH @ K_noise @ dH.T   # Q = dH * K * dH'

        # ── Actualizar covarianza ─────────────────────────────────────────
        self.Sigma = F_x @ self.Sigma @ F_x.T + Q

    # ─────────────────────────────────────────────────────────────────────────
    # EKF: Predicción con scan matching (reemplaza encoders cuando disponible)
    # ─────────────────────────────────────────────────────────────────────────

    def _ekf_predict_scanmatch(self, dx: float, dy: float, dth: float):
        """Paso de predicción usando delta de scan matching (en frame del robot).

        dx, dy: desplazamiento en el frame previo del robot [m]
        dth:    cambio de orientación [rad]
        """
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        n = len(self.mu)

        cos_th = math.cos(th)
        sin_th = math.sin(th)

        # Nuevo estado
        self.mu[0] = x + dx * cos_th - dy * sin_th
        self.mu[1] = y + dx * sin_th + dy * cos_th
        self.mu[2] = wrap_to_pi(th + dth)

        # Jacobiano del modelo de movimiento respecto al estado (solo filas del robot)
        F_x = np.eye(n)
        F_x[0, 2] = -dx * sin_th - dy * cos_th
        F_x[1, 2] =  dx * cos_th - dy * sin_th

        # Ruido de proceso del scan matching — mucho menor que encoders
        dist = math.sqrt(dx * dx + dy * dy)
        Q = np.zeros((n, n))
        Q[0, 0] = max(1e-5, 0.005 * dist)
        Q[1, 1] = max(1e-5, 0.005 * dist)
        Q[2, 2] = max(1e-6, 0.003 * abs(dth))

        self.Sigma = F_x @ self.Sigma @ F_x.T + Q

    # ─────────────────────────────────────────────────────────────────────────
    # EKF: Agregar nuevo landmark al estado
    # ─────────────────────────────────────────────────────────────────────────

    def _add_landmark(self, aruco_id: int, obs_range: float, obs_bearing: float):
        k = len(self.landmark_id_to_idx)
        self.landmark_id_to_idx[aruco_id] = k

        if aruco_id in KNOWN_ARUCO_MAP:
            # Posición física conocida → mínima incertidumbre
            x_L, y_L = KNOWN_ARUCO_MAP[aruco_id]
            init_cov = KNOWN_LM_COV
            self.get_logger().info(
                f'Landmark CONOCIDO ArUco {aruco_id} → idx {k} '
                f'en ({x_L:.2f}, {y_L:.2f}) [mapa físico]'
            )
        else:
            # Posición estimada desde la observación actual
            x_L = self.mu[0] + obs_range * math.cos(self.mu[2] + obs_bearing)
            y_L = self.mu[1] + obs_range * math.sin(self.mu[2] + obs_bearing)
            init_cov = 1.0
            self.get_logger().info(
                f'Nuevo landmark ArUco {aruco_id} → idx {k} '
                f'en ({x_L:.2f}, {y_L:.2f})'
            )

        self.mu = np.append(self.mu, [x_L, y_L])

        n_old = len(self.Sigma)
        n_new = n_old + 2
        Sigma_new = np.zeros((n_new, n_new))
        Sigma_new[:n_old, :n_old] = self.Sigma
        Sigma_new[n_old,   n_old]   = init_cov
        Sigma_new[n_old+1, n_old+1] = init_cov
        self.Sigma = Sigma_new

    # ─────────────────────────────────────────────────────────────────────────
    # EKF: Corrección con observación ArUco
    # ─────────────────────────────────────────────────────────────────────────

    def _ekf_update(self, aruco_id: int, obs_range: float, obs_bearing: float):
        k  = self.landmark_id_to_idx[aruco_id]
        n  = len(self.mu)

        # Índices del landmark en el vector de estado
        lx_idx = 3 + 2 * k
        ly_idx = 3 + 2 * k + 1

        x,  y,  th = self.mu[0],      self.mu[1],      self.mu[2]
        lx, ly      = self.mu[lx_idx], self.mu[ly_idx]

        dx   = lx - x
        dy   = ly - y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 1e-6:
            return

        # ── Observación esperada ─────────────────────────────────────────
        z_hat_range   = dist
        z_hat_bearing = wrap_to_pi(math.atan2(dy, dx) - th)

        # ── Innovación ───────────────────────────────────────────────────
        inn = np.array([
            obs_range   - z_hat_range,
            wrap_to_pi(obs_bearing - z_hat_bearing)
        ])

        # ── Jacobiano de la función de observación respecto al estado ─────
        # H tiene tamaño (2 x n); solo las columnas del robot (0-2) y el
        # landmark k tienen valores distintos de cero
        H = np.zeros((2, n))

        # Respecto a pose del robot
        H[0, 0] = -dx / dist
        H[0, 1] = -dy / dist
        H[0, 2] =  0.0

        H[1, 0] =  dy / (dist**2)
        H[1, 1] = -dx / (dist**2)
        H[1, 2] = -1.0

        # Respecto al landmark k
        H[0, lx_idx] =  dx / dist
        H[0, ly_idx] =  dy / dist

        H[1, lx_idx] = -dy / (dist**2)
        H[1, ly_idx] =  dx / (dist**2)

        # ── Covarianza del ruido de observación ──────────────────────────
        # Marcadores conocidos: mucho más confianza → corrección más agresiva
        r_factor = KNOWN_R_FACTOR if aruco_id in KNOWN_ARUCO_MAP else 1.0
        R_obs = np.diag([self.r_range * r_factor, self.r_bearing * r_factor])

        # ── Ganancia de Kalman ────────────────────────────────────────────
        S = H @ self.Sigma @ H.T + R_obs          # S = H*Sigma*H' + R
        K = self.Sigma @ H.T @ np.linalg.inv(S)   # K = Sigma*H'*S^-1

        # ── Actualizar estado y covarianza ────────────────────────────────
        self.mu    = self.mu + K @ inn
        self.Sigma = (np.eye(n) - K @ H) @ self.Sigma

        self.mu[2] = wrap_to_pi(self.mu[2])


    def _publish_status(self):
        msg = String()
        msg.data = (
            f'x={self.mu[0]:.3f} y={self.mu[1]:.3f} '
            f'yaw={math.degrees(self.mu[2]):.1f}deg '
            f'v={self.last_v:.3f}m/s w={self.last_w:.3f}rad/s '
            f'encR={self.vel_r:.3f} encL={self.vel_l:.3f} '
            f'landmarks={len(self.landmark_id_to_idx)} arucos_seen={self.last_aruco_count}'
        )
        self.pub_status.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Publicar estado estimado
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_state(self):
        now = self.get_clock().now().to_msg()
        x, y, th = self.mu[0], self.mu[1], self.mu[2]

        # ── Odometría SLAM ───────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'map'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, th)

        # Llenar covarianza publicada (solo los 3 primeros elementos del robot)
        n = min(3, self.Sigma.shape[0])
        idxs = [(0, 0), (0, 1), (1, 0), (1, 1), (0, 2), (2, 0), (2, 2)]
        ros_idx = [0, 1, 6, 7, 5, 30, 35]
        for (si, sj), ri in zip(idxs, ros_idx):
            if si < n and sj < n:
                odom.pose.covariance[ri] = self.Sigma[si, sj]

        self.pub_odom.publish(odom)

        # ── TF: map → base_link ──────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'map'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # ── Landmarks en RViz ────────────────────────────────────────────
        viz_markers = VizMarkerArray()
        for aruco_id, k in self.landmark_id_to_idx.items():
            lx = self.mu[3 + 2 * k]
            ly = self.mu[3 + 2 * k + 1]

            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = 'map'
            m.ns      = 'slam_landmarks'
            m.id      = aruco_id
            m.type    = Marker.CYLINDER
            m.action  = Marker.ADD
            m.pose.position.x = lx
            m.pose.position.y = ly
            m.pose.position.z = 0.1
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.2
            # Naranja para landmarks conocidos, verde para SLAM
            if aruco_id in KNOWN_ARUCO_MAP:
                m.color.r = 0.9
                m.color.g = 0.55
                m.color.b = 0.0
            else:
                m.color.r = 0.0
                m.color.g = 0.8
                m.color.b = 0.2
            m.color.a = 0.9

            viz_markers.markers.append(m)

            # Texto con el ID
            t_m = Marker()
            t_m.header  = m.header
            t_m.ns      = 'slam_landmark_labels'
            t_m.id      = aruco_id + 10000
            t_m.type    = Marker.TEXT_VIEW_FACING
            t_m.action  = Marker.ADD
            t_m.pose.position.x = lx
            t_m.pose.position.y = ly
            t_m.pose.position.z = 0.35
            t_m.scale.z = 0.12
            t_m.color.r = 1.0
            t_m.color.g = 1.0
            t_m.color.b = 1.0
            t_m.color.a = 1.0
            t_m.text    = f'ArUco {aruco_id}'
            viz_markers.markers.append(t_m)

        self.pub_map.publish(viz_markers)


    # ─────────────────────────────────────────────────────────────────────────
    # Guardar / cargar / limpiar landmarks (posiciones ArUco en el mapa)
    # ─────────────────────────────────────────────────────────────────────────

    def _svc_save_landmarks(self, _req, resp):
        try:
            os.makedirs(os.path.dirname(LANDMARKS_FILE), exist_ok=True)
            data = {}
            for aruco_id, k in self.landmark_id_to_idx.items():
                data[str(aruco_id)] = {
                    'x': float(self.mu[3 + 2 * k]),
                    'y': float(self.mu[3 + 2 * k + 1]),
                }
            with open(LANDMARKS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            resp.success = True
            resp.message = f'Guardados {len(data)} landmarks en {LANDMARKS_FILE}'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al guardar landmarks: {exc}'
            self.get_logger().error(resp.message)
        return resp

    def _load_landmarks(self):
        if not os.path.exists(LANDMARKS_FILE):
            return
        try:
            with open(LANDMARKS_FILE) as f:
                data = json.load(f)
            for id_str, pos in data.items():
                aruco_id = int(id_str)
                if aruco_id < self.landmark_id_min or aruco_id > self.landmark_id_max:
                    continue
                k = len(self.landmark_id_to_idx)
                self.landmark_id_to_idx[aruco_id] = k

                if aruco_id in KNOWN_ARUCO_MAP:
                    # Ignorar posición guardada (puede tener drift) — usar mapa físico
                    x_L, y_L = KNOWN_ARUCO_MAP[aruco_id]
                    init_cov = KNOWN_LM_COV
                else:
                    x_L, y_L = pos['x'], pos['y']
                    init_cov = 0.1

                self.mu = np.append(self.mu, [x_L, y_L])
                n_old = len(self.Sigma)
                n_new = n_old + 2
                S = np.zeros((n_new, n_new))
                S[:n_old, :n_old] = self.Sigma
                S[n_old,   n_old]   = init_cov
                S[n_old+1, n_old+1] = init_cov
                self.Sigma = S

            self.get_logger().info(
                f'Landmarks cargados desde {LANDMARKS_FILE}: '
                f'{list(self.landmark_id_to_idx.keys())}'
            )
        except Exception as exc:
            self.get_logger().warn(f'No se pudieron cargar landmarks: {exc}')

    def _svc_clear_landmarks(self, _req, resp):
        self.landmark_id_to_idx.clear()
        self.mu    = np.zeros(3)
        self.Sigma = np.eye(3) * 1e-6
        if os.path.exists(LANDMARKS_FILE):
            os.remove(LANDMARKS_FILE)
        resp.success = True
        resp.message = 'Landmarks borrados (mapa ArUco reseteado)'
        self.get_logger().info(resp.message)
        return resp


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = EkfSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.landmark_id_to_idx:
            node.get_logger().info('Guardando landmarks antes de salir...')
            try:
                os.makedirs(os.path.dirname(LANDMARKS_FILE), exist_ok=True)
                data = {}
                for aruco_id, k in node.landmark_id_to_idx.items():
                    data[str(aruco_id)] = {
                        'x': float(node.mu[3 + 2 * k]),
                        'y': float(node.mu[3 + 2 * k + 1]),
                    }
                with open(LANDMARKS_FILE, 'w') as f:
                    json.dump(data, f, indent=2)
                node.get_logger().info(
                    f'Landmarks guardados: {list(node.landmark_id_to_idx.keys())} → {LANDMARKS_FILE}'
                )
            except Exception as exc:
                node.get_logger().error(f'Error al guardar landmarks: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
