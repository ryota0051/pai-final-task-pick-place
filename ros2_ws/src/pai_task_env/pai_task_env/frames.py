"""Gazebo world 座標とロボット基準 (crane_plus_base) 座標の変換（ROS 非依存）."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class BaseFrame:
    """world 座標系における crane_plus_base の原点とヨー角."""

    origin: Vec3
    yaw: float = 0.0

    def world_to_base(self, p: Sequence[float]) -> Vec3:
        dx, dy, dz = p[0] - self.origin[0], p[1] - self.origin[1], p[2] - self.origin[2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy, -s * dx + c * dy, dz)

    def base_to_world(self, p: Sequence[float]) -> Vec3:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.origin[0] + c * p[0] - s * p[1],
                self.origin[1] + s * p[0] + c * p[1],
                self.origin[2] + p[2])
