"""把持・配置のスクリプト化デモ（ウェイポイント + min-jerk 補間）. ROS 非依存.

パイプライン検証とデモデータ生成に使う。座標は crane_plus_base 基準 [m]。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class Waypoint:
    phase: str
    x: float
    y: float
    z: float
    pitch: float
    gripper: float      # 0=開, 1=閉
    duration: float     # 直前のウェイポイントからこの点までの所要時間 [s]


@dataclass(frozen=True)
class PlanParams:
    z_approach: float = 0.08
    z_grasp: float = 0.02
    z_carry: float = 0.08
    z_place: float = 0.025
    pitch: float = math.pi / 2
    move_speed: float = 0.05        # [m/s]
    min_segment_time: float = 0.5   # [s]
    grasp_hold: float = 0.8         # 閉じてから持ち上げるまで [s]
    release_hold: float = 0.6       # 開いてから退避するまで [s]

    def randomized(self, rng: random.Random, speed_scale: Tuple[float, float],
                   height_jitter: float) -> 'PlanParams':
        """デモの多様性のため速度・経由高さを揺らす（把持・配置高さは固定）."""
        return replace(
            self,
            move_speed=self.move_speed * rng.uniform(*speed_scale),
            z_approach=self.z_approach + rng.uniform(-height_jitter, height_jitter),
            z_carry=self.z_carry + rng.uniform(-height_jitter, height_jitter),
        )


def _min_jerk(s: float) -> float:
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class PickPlacePlan:
    def __init__(self, waypoints: List[Waypoint]):
        self.waypoints = waypoints
        self._starts = []
        t = 0.0
        for w in waypoints:
            self._starts.append(t)
            t += w.duration
        self.total_duration = t

    @classmethod
    def create(cls, start: Sequence[float], obj_xy: Sequence[float], place_xy: Sequence[float],
               params: PlanParams) -> 'PickPlacePlan':
        p = params
        pts: List[Waypoint] = [Waypoint('start', start[0], start[1], start[2], p.pitch, 0.0, 0.0)]

        def move(phase, x, y, z, g):
            prev = pts[-1]
            dist = math.dist((prev.x, prev.y, prev.z), (x, y, z))
            pts.append(Waypoint(phase, x, y, z, p.pitch, g, max(dist / p.move_speed, p.min_segment_time)))

        def hold(phase, g, duration):
            prev = pts[-1]
            pts.append(Waypoint(phase, prev.x, prev.y, prev.z, p.pitch, g, duration))

        ox, oy = obj_xy
        px, py = place_xy
        move('approach', ox, oy, p.z_approach, 0.0)
        move('descend', ox, oy, p.z_grasp, 0.0)
        hold('grasp', 1.0, p.grasp_hold)
        move('lift', ox, oy, p.z_carry, 1.0)
        move('carry', px, py, p.z_carry, 1.0)
        move('lower', px, py, p.z_place, 1.0)
        hold('release', 0.0, p.release_hold)
        move('retreat', px, py, p.z_approach, 0.0)
        return cls(pts)

    def _index(self, phase: str) -> int:
        return [w.phase for w in self.waypoints].index(phase)

    def start_time_of(self, phase: str) -> float:
        """phase の区間（直前のウェイポイントから phase まで）の開始時刻."""
        i = self._index(phase)
        return self._starts[i]

    def duration_of(self, phase: str) -> float:
        return self.waypoints[self._index(phase)].duration

    def sample(self, t: float) -> Waypoint:
        """時刻 t の目標（位置は min-jerk 補間、グリッパは区間の目標値）."""
        if t >= self.total_duration:
            return self.waypoints[-1]
        for i in range(1, len(self.waypoints)):
            seg_start, w = self._starts[i], self.waypoints[i]
            if t < seg_start + w.duration:
                prev = self.waypoints[i - 1]
                s = _min_jerk(min(max((t - seg_start) / w.duration, 0.0), 1.0)) if w.duration > 0 else 1.0
                return Waypoint(w.phase,
                                prev.x + (w.x - prev.x) * s, prev.y + (w.y - prev.y) * s,
                                prev.z + (w.z - prev.z) * s, prev.pitch + (w.pitch - prev.pitch) * s,
                                w.gripper, w.duration)
        return self.waypoints[0]
