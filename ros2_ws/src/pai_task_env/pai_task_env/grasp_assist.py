"""把持アシストの attach / detach 判定（ROS 非依存）.

Ignition Gazebo では指の摩擦だけでは物体が滑落するため、
「グリッパが閉じ（関節角 >= close_threshold）かつ把持中心と物体が attach_distance 以内」で attach、
「グリッパが開いた（関節角 <= open_threshold）」で detach する。しきい値の間はヒステリシスとして状態保持。
"""
from __future__ import annotations

import math
from typing import Optional, Sequence


class GraspAssist:
    def __init__(self, close_threshold: float, open_threshold: float, attach_distance: float):
        if open_threshold >= close_threshold:
            raise ValueError('open_threshold must be smaller than close_threshold')
        self.close_threshold = close_threshold
        self.open_threshold = open_threshold
        self.attach_distance = attach_distance
        self.attached = False

    def update(self, hand_angle: float, grasp_center: Sequence[float],
               object_xyz: Sequence[float]) -> Optional[str]:
        """状態遷移があれば 'attach' / 'detach' を返す."""
        if not self.attached:
            if hand_angle >= self.close_threshold and \
                    math.dist(grasp_center, object_xyz) <= self.attach_distance:
                self.attached = True
                return 'attach'
        elif hand_angle <= self.open_threshold:
            self.attached = False
            return 'detach'
        return None

    def force_detach(self) -> Optional[str]:
        if self.attached:
            self.attached = False
            return 'detach'
        return None
