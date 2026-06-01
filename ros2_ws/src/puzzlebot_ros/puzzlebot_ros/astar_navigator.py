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


ROBOT_RADIUS  = 0.25  
V_MAX         = 0.08  
V_MIN         = 0.04  
W_MAX         = 0.70   


LOOKAHEAD_DIST = 0.45  
GOAL_RADIUS    = 0.25  
WP_RADIUS      = 0.10  


D_SAFE          = 0.25  
BUG_CONE_DEG    = 45    
BUG_MAX_SPIN_S    = 5.0   
BUG_REVERSE_S     = 0.8  
BUG_COOLDOWN_S    = 1.5  
BUG_DEADLOCK_MAX  = 3    
BUG_DEADLOCK_REV_S = 3.0  


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
        self._inflated: Optional[np.ndarray]    = None   

       
        self._goal:        Optional[Tuple[float, float]] = None
        self._actual_goal: Optional[Tuple[float, float]] = None  
        self._path:        List[Tuple[float, float]]     = []
        self._path_idx:    int  = 0
        self._active:      bool = False

    
        self._scan_ranges:      List[float] = []
        self._scan_angle_min:   float = 0.0
        self._scan_angle_incr:  float = 0.0
        self._bug_mode:         bool  = False
        self._bug_turn_dir:     int   = 1     
        self._bug_start:        float = 0.0   
        self._reversing:        bool  = False
        self._reverse_until:    float = 0.0
        self._bug_cooldown:     float = 0.0   
        self._bug_consec_count: int   = 0    
        self._bug_consec_pos:   Tuple[float, float] = (0.0, 0.0)

    
        self._rotating_in_place: bool  = False 
        self._rot_target_yaw:    float = 0.0   

  
        self._last_progress_pos: Tuple[float, float] = (0.0, 0.0)
        self._last_progress_t:   float = 0.0

       
        self._last_scan_t: float = 0.0  

        self.create_timer(0.10, self._nav_loop)   # 10 Hz
        self.get_logger().info('A* Navigator iniciado — esperando mapa y goal')



    def _pf_mo_cb(self, msg: TransformStamped):
     
        self._pf_last = _time.monotonic()
        self._apply_mo(msg)

    def _mo_cb(self, msg: TransformStamped):
  
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
        self._bug_mode          = False
        self._rotating_in_place = False
        self._bug_consec_count  = 0
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

        return None   # sin ruta

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



    def _obstacle_ahead(self) -> bool:
        if not self._scan_ranges:
            return False
        cone_rad = math.radians(BUG_CONE_DEG)
        for i, r in enumerate(self._scan_ranges):
            if not math.isfinite(r) or r <= 0.05:
                continue
            angle = self._scan_angle_min + i * self._scan_angle_incr
            if abs(wrap_to_pi(angle)) < cone_rad and r < D_SAFE:
                return True
        return False

    def _best_turn_dir(self) -> int:

        if not self._scan_ranges:
            return 1
        left_sum  = right_sum = 0.0
        left_cnt  = right_cnt = 0
        for i, r in enumerate(self._scan_ranges):
            if not math.isfinite(r) or r <= 0.1:
                continue
            angle = wrap_to_pi(self._scan_angle_min + i * self._scan_angle_incr)
            if math.radians(30) < angle < math.radians(150):
                left_sum += r;  left_cnt  += 1
            elif math.radians(-150) < angle < math.radians(-30):
                right_sum += r; right_cnt += 1
        mean_l = left_sum  / max(left_cnt,  1)
        mean_r = right_sum / max(right_cnt, 1)
        return 1 if mean_l >= mean_r else -1



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
            self.get_logger().info('Bug0: retroceso completo — replanificando')
            self._plan()
            return

  
        now = _time.monotonic()
        if self._obstacle_ahead() and now >= self._bug_cooldown:
            if not self._bug_mode:
          
                if math.hypot(self._pose_x - self._bug_consec_pos[0],
                              self._pose_y - self._bug_consec_pos[1]) < 0.40:
                    self._bug_consec_count += 1
                else:
                    self._bug_consec_count = 1
                    self._bug_consec_pos   = (self._pose_x, self._pose_y)

             
                if self._bug_consec_count >= BUG_DEADLOCK_MAX:
                    self._mark_scan_obstacles(r_infl=6)  
                    self._bug_mode         = False
                    self._reversing        = True
                    self._reverse_until    = now + BUG_DEADLOCK_REV_S
                    self._bug_cooldown     = now + BUG_DEADLOCK_REV_S + BUG_COOLDOWN_S
                    self._bug_consec_count = 0
                    self.get_logger().warn(
                        f'Bug0: deadlock ({BUG_DEADLOCK_MAX}x misma zona) — '
                        f'retroceso {BUG_DEADLOCK_REV_S:.0f}s + grid agresivo')
                    self._send_cmd(-V_MIN, 0.0)
                    return

                self._mark_scan_obstacles()  
                self._bug_mode     = True
                self._bug_turn_dir = self._best_turn_dir()
                self._bug_start    = now
                self._pub_status_txt('OBSTACLE — Bug0 activo')
                self.get_logger().warn(
                    f'Bug0: obstaculo ({self._bug_consec_count}x) — '
                    f'girando {"izq" if self._bug_turn_dir>0 else "der"}')

            if now - self._bug_start > BUG_MAX_SPIN_S:
                self._mark_scan_obstacles()   
                self._bug_mode      = False
                self._reversing     = True
                self._reverse_until = now + BUG_REVERSE_S
                self._bug_cooldown  = now + BUG_COOLDOWN_S
                self.get_logger().warn('Bug0: timeout — marcha atras para replantear')
                self._send_cmd(-V_MIN, 0.0)
                return

            self._send_cmd(0.0, W_MAX * 0.5 * self._bug_turn_dir)
            return

        if self._bug_mode:
            self._bug_mode     = False
            self._bug_cooldown = _time.monotonic() + BUG_COOLDOWN_S
            self.get_logger().info('Bug0: camino libre — replanificando')
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

    # ── Helpers ───────────────────────────────────────────────────────────────

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
        self.get_logger().info(f'Bug0: {count} celdas dinamicas marcadas en grid')

    def _send_cmd(self, v: float, w: float):
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
