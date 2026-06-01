

import math

from geometry_msgs.msg import Quaternion


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_lerp(a0: float, a1: float, ratio: float) -> float:
    delta = angle_normalize(a1 - a0)
    return angle_normalize(a0 + ratio * delta)


def bresenham(x0: int, y0: int, x1: int, y1: int):
   
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
