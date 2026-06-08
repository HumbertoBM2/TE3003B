
from collections import deque
from typing import Optional

from nav_msgs.msg import Odometry

from .slam_math import angle_lerp, stamp_to_sec, yaw_from_quaternion
from .slam_types import Pose2D


class OdometryBuffer:
    def __init__(self, buffer_sec: float, max_lookup_age: float):
        self._buffer_sec = buffer_sec
        self._max_lookup_age = max_lookup_age
        self._poses = deque()

    @property
    def has_pose(self) -> bool:
        return bool(self._poses)

    @property
    def latest_pose(self) -> Optional[Pose2D]:
        if not self._poses:
            return None
        return self._poses[-1][1]

    def add(self, msg: Odometry) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        pose = Pose2D(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self._poses.append((stamp, pose))

        cutoff = stamp - self._buffer_sec
        while self._poses and self._poses[0][0] < cutoff:
            self._poses.popleft()

    def lookup(self, stamp) -> Optional[Pose2D]:
        if not self._poses:
            return None

        target = stamp_to_sec(stamp)
        if target <= 0.0:
            return self._poses[-1][1]

        first_t, first_pose = self._poses[0]
        last_t, last_pose = self._poses[-1]

        if target <= first_t:
            return first_pose if abs(first_t - target) <= self._max_lookup_age else None
        if target >= last_t:
            return last_pose if abs(target - last_t) <= self._max_lookup_age else None

        for index in range(1, len(self._poses)):
            before_t, before_pose = self._poses[index - 1]
            after_t, after_pose = self._poses[index]
            if before_t <= target <= after_t:
                dt = after_t - before_t
                if dt <= 1e-9:
                    return after_pose
                ratio = (target - before_t) / dt
                return Pose2D(
                    before_pose.x + ratio * (after_pose.x - before_pose.x),
                    before_pose.y + ratio * (after_pose.y - before_pose.y),
                    angle_lerp(before_pose.yaw, after_pose.yaw, ratio),
                )

        return None
