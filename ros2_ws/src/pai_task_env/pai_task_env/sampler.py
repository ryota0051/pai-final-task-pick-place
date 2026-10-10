"""物体スポーン位置のサンプラ（ROS 非依存）. 座標は crane_plus_base 基準 [m]."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Mapping, Tuple

Range = Tuple[float, float]


class SamplingError(RuntimeError):
    pass


@dataclass(frozen=True)
class ObjectSampler:
    x: Range
    y: Range
    r: Range
    place_xy: Tuple[float, float]
    min_dist_from_place: float
    max_attempts: int = 1000

    @classmethod
    def from_dict(cls, cfg: Mapping) -> 'ObjectSampler':
        def rng(k):
            return (float(cfg[k][0]), float(cfg[k][1]))
        return cls(x=rng('x'), y=rng('y'), r=rng('r'),
                   place_xy=(float(cfg['place_xy'][0]), float(cfg['place_xy'][1])),
                   min_dist_from_place=float(cfg['min_dist_from_place']),
                   max_attempts=int(cfg.get('max_attempts', 1000)))

    def is_valid(self, x: float, y: float) -> bool:
        rad = math.hypot(x, y)
        far = math.hypot(x - self.place_xy[0], y - self.place_xy[1]) >= self.min_dist_from_place
        return self.r[0] <= rad <= self.r[1] and far

    def sample(self, rng: random.Random) -> Tuple[float, float]:
        """x, y 範囲から一様サンプルし、半径範囲・置き場所からの距離制約を満たすまで棄却する."""
        for _ in range(self.max_attempts):
            x = rng.uniform(*self.x)
            y = rng.uniform(*self.y)
            if self.is_valid(x, y):
                return (x, y)
        raise SamplingError(f'no valid sample in {self.max_attempts} attempts')


def sample_positions(sampler: ObjectSampler, n: int, seed: int):
    """評価用に、同じ条件で再現可能な物体位置リストを生成する."""
    rng = random.Random(seed)
    return [sampler.sample(rng) for _ in range(n)]
