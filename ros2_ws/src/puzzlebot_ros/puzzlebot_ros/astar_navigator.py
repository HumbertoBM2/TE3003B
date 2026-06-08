#!/usr/bin/env python3


import heapq
import math
import time as _time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .my_math import euler_from_quaternion, wrap_to_pi


ROBOT_RADIUS      = 0.27   
FORK_CLEARANCE_M  = 0.12  
ANTENNA_REAR_M    = 0.045  
V_MAX         = 0.07   
V_MIN         = 0.04  
W_MAX         = 0.45   
V_LIN_STEP    = 0.025  
W_ANG_STEP    = 0.18   

LOOKAHEAD_DIST = 0.45 
GOAL_RADIUS    = 0.25  
WP_RADIUS      = 0.10  


D_OBS                 = 0.40   
D_FOLLOW              = 0.28   
BUG_CONE_DEG          = 45    
BUG1_CIRCUM_TIMEOUT_S = 30.0   
BUG_REVERSE_S         = 1.0   


class _Bug2:


    FREE        = 0
    WALL_FOLLOW = 1
    RECOVERY    = 2

    _CONFIRM_N    = 3    
    _LEAVE_TOL_M  = 0.20  
    _LEAVE_PROG_M = 0.25  

    def __init__(self, node) -> None:
        self._node            = node
        self._state: int      = self.FREE
        self._hit_x: float    = 0.0
        self._hit_y: float    = 0.0
        self._hit_d_goal: float = math.inf
        self._follow_side: int  = -1   
        self._prev_wall_err: float = 0.0
        self._start_t: float       = 0.0   
        self._recovery_until: float = 0.0
        self._obs_count: int        = 0


    def update(
        self,
        scan_ranges: List[float],
        angle_min:   float,
        angle_incr:  float,
        robot_x:     float,
        robot_y:     float,
        robot_yaw:   float,
        goal_x:      float,
        goal_y:      float,
    ) -> Optional[Tuple[float, float]]:
        """Return (v, w) when Bug2 active, None when path is clear."""
        now = _time.monotonic()

        if self._state == self.FREE:
            if self._obstacle_ahead(scan_ranges, angle_min, angle_incr):
                self._obs_count += 1
            else:
                self._obs_count = 0
            if self._obs_count >= self._CONFIRM_N:
                self._obs_count = 0
                d_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
                self._hit_x = robot_x
                self._hit_y = robot_y
                self._hit_d_goal = d_goal
                dist_right = self._sector_min(
                    scan_ranges, angle_min, angle_incr, -90.0, -10.0)
                dist_left  = self._sector_min(
                    scan_ranges, angle_min, angle_incr,  10.0,  90.0)
                self._follow_side = -1 if dist_left >= dist_right else 1
                self._prev_wall_err = 0.0
                side_name = 'DER' if self._follow_side == -1 else 'IZQ'
                self._node.get_logger().warn(
                    f'Bug2: obstáculo en ({robot_x:.2f},{robot_y:.2f}) '
                    f'd_goal={d_goal:.2f}m | pared {side_name}')
                front_min = self._sector_min(
                    scan_ranges, angle_min, angle_incr, -30.0, 30.0)
                if front_min < D_OBS * 0.6:
                    self._recovery_until = now + BUG_REVERSE_S
                    self._state = self.RECOVERY
                    return (-V_MIN, 0.0)
                self._start_t = now
                self._state   = self.WALL_FOLLOW
                return self._wall_follow_cmd(scan_ranges, angle_min, angle_incr)
            return None

        if self._state == self.RECOVERY:
            if now < self._recovery_until:
                return (-V_MIN, 0.0)
            self._node.get_logger().info('Bug2: recovery → WALL_FOLLOW')
            self._start_t = now
            self._state   = self.WALL_FOLLOW
            return self._wall_follow_cmd(scan_ranges, angle_min, angle_incr)

        if self._state == self.WALL_FOLLOW:
            if self._check_leave(robot_x, robot_y, goal_x, goal_y):
                self._node.get_logger().info('Bug2: M-line cruzada → FREE')
                self._state = self.FREE
                return None
            return self._wall_follow_cmd(scan_ranges, angle_min, angle_incr)

        return None

    def reset(self) -> None:
        self._state         = self.FREE
        self._obs_count     = 0
        self._prev_wall_err = 0.0

    @property
    def is_active(self) -> bool:
        return self._state != self.FREE

    @property
    def is_timed_out(self) -> bool:
        return (self._state == self.WALL_FOLLOW and
                _time.monotonic() - self._start_t > BUG1_CIRCUM_TIMEOUT_S)


    def _obstacle_ahead(self, ranges, angle_min, angle_incr) -> bool:
        if not ranges:
            return False
        cone = math.radians(BUG_CONE_DEG)
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.05:
                continue
            a = math.atan2(math.sin(angle_min + i * angle_incr),
                           math.cos(angle_min + i * angle_incr))
            if abs(a) <= cone and r < D_OBS:
                return True
        return False

    def _sector_min(self, ranges, angle_min, angle_incr,
                    a_lo_deg: float, a_hi_deg: float) -> float:
        a_lo = math.radians(a_lo_deg)
        a_hi = math.radians(a_hi_deg)
        best = math.inf
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.05:
                continue
            a = math.atan2(math.sin(angle_min + i * angle_incr),
                           math.cos(angle_min + i * angle_incr))
            if a_lo <= a <= a_hi:
                best = min(best, r)
        return best if math.isfinite(best) else D_OBS * 2.0

    def _wall_follow_cmd(self, ranges, angle_min, angle_incr) -> Tuple[float, float]:
        """PD wall-follow. follow_side=-1 → pared derecha; +1 → pared izquierda."""
        if self._follow_side == -1:
            side_dist = self._sector_min(ranges, angle_min, angle_incr, -90.0, -10.0)
        else:
            side_dist = self._sector_min(ranges, angle_min, angle_incr,  10.0,  90.0)
        front = self._sector_min(ranges, angle_min, angle_incr, -30.0, 30.0)
        if front < D_OBS:
            return 0.0, -self._follow_side * W_MAX * 0.7

        err   = side_dist - D_FOLLOW
        d_err = err - self._prev_wall_err
        self._prev_wall_err = err


        w = float(max(-W_MAX * 0.7, min(W_MAX * 0.7,
                      self._follow_side * (1.8 * err + 0.2 * d_err))))
        return V_MAX * 0.6, w

    def _check_leave(self, rx: float, ry: float,
                     gx: float, gy: float) -> bool:
        lx = gx - self._hit_x
        ly = gy - self._hit_y
        line_len = math.hypot(lx, ly)
        if line_len < 0.10:
            return True 
        dx = rx - self._hit_x
        dy = ry - self._hit_y
        t  = (dx * lx + dy * ly) / (line_len * line_len)
        if t < 0.05:
            return False  
        perp_x = dx - t * lx
        perp_y = dy - t * ly
        if math.hypot(perp_x, perp_y) > self._LEAVE_TOL_M:
            return False 
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        return dist_to_goal < (self._hit_d_goal - self._LEAVE_PROG_M)


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('astar_navigator')

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',    qos_be)
        self._pub_status = self.create_publisher(String, '/nav/status', qos_rel)
        self._pub_path   = self.create_publisher(Path,   '/nav/path',   qos_rel)

        self.create_subscription(Odometry,          '/odom',            self._odom_cb,   qos_be)
        self.create_subscription(TransformStamped,  '/pf/map_to_odom',  self._pf_mo_cb,  qos_rel)
        self.create_subscription(TransformStamped,  '/map_to_odom',     self._mo_cb,     qos_rel)
        self.create_subscription(OccupancyGrid,     '/map',             self._map_cb,    qos_latched)
        self.create_subscription(PoseStamped,       '/nav/goal',        self._goal_cb,   qos_rel)
        self.create_subscription(LaserScan,         '/scan_stamped',    self._scan_cb,   qos_be)

        self._pose_x   = 0.0
        self._pose_y   = 0.0
        self._pose_yaw = 0.0
        self._mo_x     = 0.0
        self._mo_y     = 0.0
        self._mo_yaw   = 0.0
        self._odom_ok  = False
        self._pf_last  = 0.0   

        self._map:      Optional[OccupancyGrid] = None
        self._inflated: Optional[np.ndarray]    = None   # 0=libre, 1=bloqueado

        self._goal:        Optional[Tuple[float, float]] = None
        self._actual_goal: Optional[Tuple[float, float]] = None  # último wp planificado
        self._path:        List[Tuple[float, float]]     = []
        self._path_idx:    int  = 0
        self._active:      bool = False

        self._scan_ranges:     List[float] = []
        self._scan_angle_min:  float = 0.0
        self._scan_angle_incr: float = 0.0

        self._bug2            = _Bug2(self)
        self._bug1_was_active = False
        self._reversing       = False
        self._reverse_until   = 0.0

        self._prev_v: float = 0.0
        self._prev_w: float = 0.0

        self._rotating_in_place: bool  = False  # True mientras |alpha| > 90°
        self._rot_target_yaw:    float = 0.0    # heading cacheado al entrar en rotación

        self._last_progress_pos: Tuple[float, float] = (0.0, 0.0)
        self._last_progress_t:   float = 0.0

        self._last_scan_t: float = 0.0   # monotonic del último scan recibido

        self.create_timer(0.10, self._nav_loop)   # 10 Hz
        self.get_logger().info('A* Navigator iniciado — esperando mapa y goal')


    def _pf_mo_cb(self, msg: TransformStamped):
        """Actualización desde el filtro de partículas (fuente preferida)."""
        self._pf_last = _time.monotonic()
        self._apply_mo(msg)

    def _mo_cb(self, msg: TransformStamped):
        """Actualización ArUco (fallback si el PF no está corriendo)."""
        if _time.monotonic() - self._pf_last > 0.5:
            self._apply_mo(msg)

    def _apply_mo(self, msg: TransformStamped):
        self._mo_x = msg.transform.translation.x
        self._mo_y = msg.transform.translation.y
        q = msg.transform.rotation
        self._mo_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_cb(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        _, _, yaw_odom = euler_from_quaternion(msg.pose.pose.orientation)

        c = math.cos(self._mo_yaw)
        s = math.sin(self._mo_yaw)
        self._pose_x   = self._mo_x + c * ox - s * oy
        self._pose_y   = self._mo_y + s * ox + c * oy
        self._pose_yaw = wrap_to_pi(yaw_odom + self._mo_yaw)
        self._odom_ok  = True

    def _map_cb(self, msg: OccupancyGrid):
        self._map      = msg
        self._inflated = self._inflate(msg)
        self.get_logger().info(
            f'Mapa recibido: {msg.info.width}x{msg.info.height} | '
            f'res={msg.info.resolution:.3f} m')

    def _goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        if abs(gx) > 1000 or abs(gy) > 1000:
            self.get_logger().info('Cancelacion recibida — deteniendo')
            self._active       = False
            self._bug_mode     = False
            self._path         = []
            self._actual_goal  = None
            self._send_cmd(0.0, 0.0)
            self._pub_status_txt('IDLE')
            return

        self._goal              = (gx, gy)
        self._active            = True
        self._bug2.reset()
        self._bug1_was_active   = False
        self._reversing         = False
        self._rotating_in_place = False
        self._last_progress_pos = (self._pose_x, self._pose_y)
        self._last_progress_t   = _time.monotonic()
        self.get_logger().info(f'Goal recibido: ({gx:.2f}, {gy:.2f})')
        self._plan()

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_t     = _time.monotonic()
        self._scan_ranges     = list(msg.ranges)
        self._scan_angle_min  = msg.angle_min
        self._scan_angle_incr = msg.angle_increment


    def _inflate(self, msg: OccupancyGrid) -> np.ndarray:
        """Dilata celdas ocupadas en el grid por ROBOT_RADIUS."""
        res  = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)

        occ = (grid > 50).astype(np.uint8)

        r = int(math.ceil(ROBOT_RADIUS / res))

        inflated = np.zeros_like(occ)
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc <= r * r:
                    shifted = np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
                    inflated = np.maximum(inflated, shifted)
        return inflated

    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        if self._map is None:
            return 0, 0
        res = self._map.info.resolution
        ox  = self._map.info.origin.position.x
        oy  = self._map.info.origin.position.y
        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        return row, col

    def _grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        if self._map is None:
            return 0.0, 0.0
        res = self._map.info.resolution
        ox  = self._map.info.origin.position.x
        oy  = self._map.info.origin.position.y
        return ox + (col + 0.5) * res, oy + (row + 0.5) * res

    def _nearest_free(self, row: int, col: int) -> Tuple[int, int]:
        """Busca la celda libre más cercana a (row, col) en espiral."""
        if self._inflated is None:
            return row, col
        h, w = self._inflated.shape
        for radius in range(0, 12):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < h and 0 <= nc < w and self._inflated[nr, nc] == 0:
                        return nr, nc
        return row, col

    def _astar(
        self,
        start: Tuple[int, int],
        goal:  Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        if self._inflated is None:
            return None

        h_grid, w_grid = self._inflated.shape

        def is_free(r: int, c: int) -> bool:
            return 0 <= r < h_grid and 0 <= c < w_grid and self._inflated[r, c] == 0

        start = self._nearest_free(*start)
        goal  = self._nearest_free(*goal)

        if start == goal:
            return [start]

        DIRS  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        COSTS = [1.0,    1.0,  1.0,   1.0,   1.414,   1.414,  1.414,  1.414]

        open_set: List[Tuple[float, Tuple[int,int]]] = []
        heapq.heappush(open_set, (0.0, start))
        came_from: Dict[Tuple[int,int], Tuple[int,int]] = {}
        g: Dict[Tuple[int,int], float] = {start: 0.0}

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                path = []
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                path.reverse()
                return path

            for (dr, dc), cost in zip(DIRS, COSTS):
                nb = (cur[0] + dr, cur[1] + dc)
                if not is_free(*nb):
                    continue
                ng = g[cur] + cost
                if nb not in g or ng < g[nb]:
                    g[nb] = ng
                    h_val = math.sqrt((nb[0]-goal[0])**2 + (nb[1]-goal[1])**2)
                    heapq.heappush(open_set, (ng + h_val, nb))
                    came_from[nb] = cur

        return None   

    def _smooth(self, grid_path: List[Tuple[int,int]]) -> List[Tuple[float,float]]:
        """Convierte grid → coordenadas mundo y elimina puntos colineales."""
        world = [self._grid_to_world(r, c) for r, c in grid_path]
        if len(world) <= 2:
            return world

        out = [world[0]]
        for i in range(1, len(world) - 1):
            p0, p1, p2 = out[-1], world[i], world[i + 1]
            cross = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
            if abs(cross) > 0.005:   
                out.append(p1)
        out.append(world[-1])
        return out

    def _plan(self):
        self._rotating_in_place = False   
        if self._map is None:
            self._pub_status_txt('ERROR: sin mapa')
            return
        if self._goal is None:
            return

        sr, sc = self._world_to_grid(self._pose_x, self._pose_y)
        gr, gc = self._world_to_grid(self._goal[0], self._goal[1])

        self._pub_status_txt('PLANNING')
        self.get_logger().info(f'A*: ({sr},{sc}) → ({gr},{gc})')

        grid_path = self._astar((sr, sc), (gr, gc))
        if grid_path is None:
            self._pub_status_txt('ERROR: sin ruta')
            self.get_logger().warn('A*: no se encontro ruta')
            self._active = False
            self._send_cmd(0.0, 0.0)  
            return

        self._path        = self._smooth(grid_path)
        self._path_idx    = 0
        self._actual_goal = self._path[-1] if self._path else None
        self._publish_path()
        self._pub_status_txt(f'NAVIGATING waypoints={len(self._path)}')
        self.get_logger().info(f'A*: {len(self._path)} waypoints')

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wx, wy in self._path:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)


    def _lookahead_point(self) -> Optional[Tuple[float, float]]:
        if not self._path:
            return None

        min_d = float('inf')
        closest = self._path_idx
        for i in range(self._path_idx, len(self._path)):
            d = math.hypot(self._path[i][0] - self._pose_x,
                           self._path[i][1] - self._pose_y)
            if d < min_d:
                min_d = d
                closest = i
        self._path_idx = closest

        acc = 0.0
        for i in range(closest, len(self._path) - 1):
            seg_dx = self._path[i+1][0] - self._path[i][0]
            seg_dy = self._path[i+1][1] - self._path[i][1]
            seg_l  = math.hypot(seg_dx, seg_dy)
            if acc + seg_l >= LOOKAHEAD_DIST:
                t  = (LOOKAHEAD_DIST - acc) / seg_l
                return (self._path[i][0] + t * seg_dx,
                        self._path[i][1] + t * seg_dy)
            acc += seg_l

        return self._path[-1]


    def _nav_loop(self):
        if not self._odom_ok or not self._active or self._goal is None:
            return

        # Watchdog LiDAR: sin datos de scan por >6s → parar y avisar
        if self._last_scan_t > 0.0 and _time.monotonic() - self._last_scan_t > 6.0:
            self._send_cmd(0.0, 0.0)
            self._pub_status_txt('ERROR: LiDAR sin datos — detenido')
            self.get_logger().error(
                'LiDAR sin datos >6s — navegacion detenida por seguridad')
            self._active = False
            return


        check = self._actual_goal if self._actual_goal else self._goal
        dist_goal = min(
            math.hypot(self._goal[0] - self._pose_x, self._goal[1] - self._pose_y),
            math.hypot(check[0]      - self._pose_x, check[1]      - self._pose_y),
        )
        if dist_goal < GOAL_RADIUS:
            self._send_cmd(0.0, 0.0)
            self._active = False
            self._bug_mode = False
            self._pub_status_txt('ARRIVED')
            self.get_logger().info('Goal alcanzado')
            return

        if self._reversing:
            if _time.monotonic() < self._reverse_until:
                self._send_cmd(-V_MIN, 0.0)
                return
            self._reversing = False
            self._bug2.reset()
            self.get_logger().info('Bug2: retroceso completo — replanificando')
            self._plan()
            return

        now = _time.monotonic()
        bug2_cmd = self._bug2.update(
            self._scan_ranges, self._scan_angle_min, self._scan_angle_incr,
            self._pose_x, self._pose_y, self._pose_yaw,
            self._goal[0], self._goal[1],
        )
        if bug2_cmd is not None:
            if not self._bug1_was_active:
                self._bug1_was_active = True
            if self._bug2.is_timed_out:
                self.get_logger().warn(
                    f'Bug2: timeout {BUG1_CIRCUM_TIMEOUT_S:.0f}s — '
                    'marcando grid y retrocediendo')
                self._mark_scan_obstacles(r_infl=6)
                self._reversing     = True
                self._reverse_until = now + BUG_REVERSE_S
                self._send_cmd(-V_MIN, 0.0)
            else:
                v, w = bug2_cmd
                self._send_cmd(v, w)
                self._pub_status_txt('OBSTACLE — Bug2 activo')
            return

        if self._bug1_was_active:
            self._bug1_was_active = False
            self._mark_scan_obstacles()
            self.get_logger().info('Bug2: camino libre — replanificando')
            self._plan()
            return

        if not self._path:
            self._plan()
            return

        now_pp = _time.monotonic()
        prog_d = math.hypot(self._pose_x - self._last_progress_pos[0],
                            self._pose_y - self._last_progress_pos[1])
        if prog_d >= 0.15:
            self._last_progress_pos = (self._pose_x, self._pose_y)
            self._last_progress_t   = now_pp
        elif now_pp - self._last_progress_t > 15.0:
            self.get_logger().warn('Sin progreso 15s — forzando replaneacion')
            self._rotating_in_place = False
            self._last_progress_pos = (self._pose_x, self._pose_y)
            self._last_progress_t   = now_pp
            self._plan()
            return

        target = self._lookahead_point()
        if target is None:
            target = self._goal

        dx    = target[0] - self._pose_x
        dy    = target[1] - self._pose_y
        alpha = wrap_to_pi(math.atan2(dy, dx) - self._pose_yaw)

   
        if not self._rotating_in_place and abs(alpha) > math.pi / 2.0:
            self._rotating_in_place = True
            self._rot_target_yaw    = math.atan2(dy, dx)
            self.get_logger().info(
                f'Rotacion: yaw={math.degrees(self._pose_yaw):.0f}deg '
                f'-> target={math.degrees(self._rot_target_yaw):.0f}deg')

        if self._rotating_in_place:
            ang_err = wrap_to_pi(self._rot_target_yaw - self._pose_yaw)
            if abs(ang_err) < math.radians(25):
                self._rotating_in_place = False
                self.get_logger().info(
                    f'Rotacion completa: yaw={math.degrees(self._pose_yaw):.0f}deg')
            else:
                self._send_cmd(0.0, math.copysign(W_MAX * 0.7, ang_err))
                self._pub_status_txt(f'ROTATING {math.degrees(ang_err):.0f}deg')
                return

    
        v = V_MAX * max(0.3, math.cos(alpha))
        v = max(V_MIN, min(V_MAX, v))
        w = 2.0 * v * math.sin(alpha) / max(LOOKAHEAD_DIST, 0.01)
        w = max(-W_MAX, min(W_MAX, w))

        self._send_cmd(v, w)
        self._pub_status_txt(
            f'NAVIGATING dist={dist_goal:.2f} wp={self._path_idx}/{len(self._path)}')


    def _mark_scan_obstacles(self, r_infl: int = 3):
    
        if self._inflated is None or self._map is None or not self._scan_ranges:
            return
        res = self._map.info.resolution
        ox  = self._map.info.origin.position.x
        oy  = self._map.info.origin.position.y
        h, w_grid = self._inflated.shape
        count  = 0
        for i, r in enumerate(self._scan_ranges):
            if not math.isfinite(r) or r < 0.10 or r > 1.0:
                continue
            angle  = self._scan_angle_min + i * self._scan_angle_incr + self._pose_yaw
            obs_x  = self._pose_x + r * math.cos(angle)
            obs_y  = self._pose_y + r * math.sin(angle)
            gr     = int((obs_y - oy) / res)
            gc     = int((obs_x - ox) / res)
            for dr in range(-r_infl, r_infl + 1):
                for dc in range(-r_infl, r_infl + 1):
                    if dr * dr + dc * dc > r_infl * r_infl:
                        continue
                    nr, nc = gr + dr, gc + dc
                    if 0 <= nr < h and 0 <= nc < w_grid and self._inflated[nr, nc] == 0:
                        self._inflated[nr, nc] = 1
                        count += 1
        self.get_logger().info(f'Bug1: {count} celdas dinamicas marcadas en grid')

    def _send_cmd(self, v: float, w: float):
        dv = max(-V_LIN_STEP, min(V_LIN_STEP, v - self._prev_v))
        dw = max(-W_ANG_STEP, min(W_ANG_STEP, w - self._prev_w))
        v = self._prev_v + dv
        w = self._prev_w + dw
        self._prev_v = v
        self._prev_w = w
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self._pub_cmd.publish(msg)

    def _pub_status_txt(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigator()
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
