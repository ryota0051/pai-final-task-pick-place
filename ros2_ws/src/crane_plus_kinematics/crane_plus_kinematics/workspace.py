"""手先目標を作業空間内にクリップするユーティリティ（ROS 非依存）."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

Range = Tuple[float, float]


def _clamp(v: float, rng: Range) -> float:
    return min(max(v, rng[0]), rng[1])


@dataclass(frozen=True)
class Workspace:
    """直方体 (x, y, z) と水平半径 r の範囲で定義される作業空間 [m]."""

    x: Range
    y: Range
    z: Range
    r: Range

    @classmethod
    def from_dict(cls, cfg: Mapping[str, Sequence[float]]) -> 'Workspace':
        def rng(k):
            return (float(cfg[k][0]), float(cfg[k][1]))
        return cls(x=rng('x'), y=rng('y'), z=rng('z'), r=rng('r'))

    def contains(self, x: float, y: float, z: float) -> bool:
        return self.clip(x, y, z) == (x, y, z)

    def clip(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        cx, cy, cz = _clamp(x, self.x), _clamp(y, self.y), _clamp(z, self.z)
        rad = math.hypot(cx, cy)
        target = _clamp(rad, self.r)
        if rad < 1e-12:
            cx, cy = target, 0.0
        elif target != rad:
            cx, cy = cx * target / rad, cy * target / rad
        return (cx, cy, cz)
