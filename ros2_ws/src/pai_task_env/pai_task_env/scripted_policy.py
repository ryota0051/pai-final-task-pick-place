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
    # 下降後、開いたまま待ってから閉じる [s]。腕は指令に遅れて追従するので、目標高さに着いた瞬間に閉じると
    # 実際はまだ 2〜3cm 上を下降中で、水平にずれていると指が物体を押し出す（左前で失敗が集中した原因, 2026-10-13）。0 で従来どおり
    grasp_settle: float = 0.0
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


def random_start_offset(rng: random.Random, max_xyz: Sequence[float]) -> Tuple[float, float, float]:
    """デモの開始姿勢（ホーム手先位置からのずれ）を各軸 ±max の一様分布で決める.

    方策が「静止したホーム姿勢から始まる」ことに依存しないよう、開始姿勢にばらつきを与える（実験8）。
    全軸 0 なら乱数を消費しない（既存設定での seed の再現性を保つ）。
    """
    if not any(m > 0 for m in max_xyz):
        return (0.0, 0.0, 0.0)
    return tuple(rng.uniform(-m, m) for m in max_xyz)


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
        if p.grasp_settle > 0:
            hold('settle', 0.0, p.grasp_settle)
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

    def end_time_of(self, phase: str) -> float:
        return self.start_time_of(phase) + self.duration_of(phase)

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


@dataclass(frozen=True)
class DisturbanceParams:
    """DART（Laskey et al., 2017）の外乱. 実行する手先目標にだけ足し、ラベル（収録する指令）は外乱なしにする.

    計画どおりの軌道から外れた状態と、そこから戻す正解の指令の組をデモに含めるため（実験13）。
    """
    max_xy: float = 0.0         # 水平の外乱の最大（半径）[m]。max_xy, max_z とも 0 なら外乱なし
    max_z: float = 0.0          # 鉛直の外乱の最大 [m]
    period: float = 0.5         # 外乱の目標値を引き直す間隔 [s]（間は min-jerk で滑らかにつなぐ）
    phases: Tuple[str, ...] = ('approach', 'descend')   # 外乱を入れる区間（最初のフェーズの開始〜最後のフェーズの終了）
    fade_time: float = 0.4      # 区間の終わりに向けて外乱を 0 に戻す時間 [s]
    end_margin: float = 0.2     # 最後のフェーズの終了のこの時間前までに外乱を 0 にする [s]（閉じる前に腕を追従させる）

    @property
    def enabled(self) -> bool:
        return self.max_xy > 0 or self.max_z > 0


class Disturbance:
    """計画の時刻 t に対する手先目標の外乱 (dx, dy, dz) [m].

    区間の始まりは 0 から、period ごとに引き直した目標値（水平は半径 max_xy の円内で一様）を min-jerk でつなぎ、
    区間の終わり fade_time かけて 0 に戻す。区間外は 0。
    """

    def __init__(self, plan: PickPlacePlan, params: DisturbanceParams, rng: random.Random):
        self.params = params
        phases = [w.phase for w in plan.waypoints]
        unknown = [ph for ph in params.phases if ph not in phases]
        if unknown:
            raise ValueError(f'unknown phase for disturbance: {unknown} (plan has {phases})')
        self.start = plan.start_time_of(params.phases[0])
        self.end = plan.end_time_of(params.phases[-1]) - params.end_margin
        self.knots: List[Tuple[float, float, float]] = []
        if not params.enabled or self.end <= self.start:
            return
        n = int(math.ceil((self.end - self.start) / params.period))
        self.knots.append((0.0, 0.0, 0.0))
        for _ in range(n):
            r = params.max_xy * math.sqrt(rng.random())
            a = rng.uniform(-math.pi, math.pi)
            dz = rng.uniform(-params.max_z, params.max_z) if params.max_z > 0 else 0.0
            self.knots.append((r * math.cos(a), r * math.sin(a), dz))

    @property
    def enabled(self) -> bool:
        return bool(self.knots)

    def offset(self, t: float) -> Tuple[float, float, float]:
        if not self.knots or t < self.start or t >= self.end:
            return (0.0, 0.0, 0.0)
        u = (t - self.start) / self.params.period
        i = min(int(u), len(self.knots) - 2)
        s = _min_jerk(min(max(u - i, 0.0), 1.0))
        a, b = self.knots[i], self.knots[i + 1]
        fade = self.params.fade_time
        gain = _min_jerk(min((self.end - t) / fade, 1.0)) if fade > 0 else 1.0
        return tuple(gain * (a[k] + (b[k] - a[k]) * s) for k in range(3))
