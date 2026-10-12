"""タスク成功判定（ROS 非依存）.

- pick モード: 物体が一度持ち上げられ (lifted)、その後置き場所ゾーン内のテーブル上に
  hold_time 以上連続して静置されたら成功。
- reach モード（把持が不安定な場合のフォールバック）: 手先が物体に reach_threshold 以内で
  hold_time 以上連続したら成功。
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple


class SuccessTracker:
    def __init__(self, place_xy: Tuple[float, float], place_radius: float, rest_z: float,
                 z_tolerance: float, lift_height: float, hold_time: float,
                 mode: str = 'pick', reach_threshold: float = 0.02):
        if mode not in ('pick', 'reach'):
            raise ValueError(f'unknown mode: {mode}')
        self.place_xy = place_xy
        self.place_radius = place_radius
        self.rest_z = rest_z
        self.z_tolerance = z_tolerance
        self.lift_height = lift_height
        self.hold_time = hold_time
        self.mode = mode
        self.reach_threshold = reach_threshold
        self.reset()

    def set_place(self, place_xy: Tuple[float, float]) -> None:
        """置き場所を変える（置き場所をエピソードごとにランダム化する場合）."""
        self.place_xy = place_xy

    def reset(self) -> None:
        self.lifted = False
        self.success = False
        self._hold_start: Optional[float] = None

    def _in_place_zone(self, obj: Sequence[float]) -> bool:
        d = math.hypot(obj[0] - self.place_xy[0], obj[1] - self.place_xy[1])
        return d <= self.place_radius and abs(obj[2] - self.rest_z) <= self.z_tolerance

    def _hold(self, t: float, cond: bool) -> None:
        if not cond:
            self._hold_start = None
            return
        if self._hold_start is None:
            self._hold_start = t
        if t - self._hold_start >= self.hold_time - 1e-9:
            self.success = True

    def update(self, t: float, obj_xyz: Sequence[float],
               ee_xyz: Optional[Sequence[float]] = None) -> dict:
        if obj_xyz[2] - self.rest_z >= self.lift_height:
            self.lifted = True
        in_zone = self._in_place_zone(obj_xyz)
        if not self.success:
            if self.mode == 'pick':
                self._hold(t, self.lifted and in_zone)
            elif ee_xyz is not None:
                self._hold(t, math.dist(ee_xyz, obj_xyz) <= self.reach_threshold)
        return {'lifted': self.lifted, 'in_place_zone': in_zone, 'success': self.success}
