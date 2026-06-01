

import math

from .slam_math import angle_normalize
from .slam_types import Pose2D


class KeyframeManager:
    def __init__(self, enabled: bool, min_translation: float, min_rotation: float):
        self._enabled = enabled
        self._min_translation = min_translation
        self._min_rotation = min_rotation
        self._last_pose = None

    def should_integrate(self, pose: Pose2D) -> bool:
        if not self._enabled:
            return True
        if self._last_pose is None:
            self._last_pose = pose
            return True

        dx = pose.x - self._last_pose.x
        dy = pose.y - self._last_pose.y
        distance = math.hypot(dx, dy)
        dyaw = abs(angle_normalize(pose.yaw - self._last_pose.yaw))
        if distance >= self._min_translation or dyaw >= self._min_rotation:
            self._last_pose = pose
            return True
        return False
