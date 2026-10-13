"""物体・置き場所の位置のサンプラ（ROS 非依存）. 座標は crane_plus_base 基準 [m]."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

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

    def is_valid(self, x: float, y: float, place_xy: Optional[Tuple[float, float]] = None) -> bool:
        """place_xy を省略すると設定の固定位置（置き場所をランダム化しない場合）からの距離で判定する."""
        px, py = place_xy if place_xy is not None else self.place_xy
        rad = math.hypot(x, y)
        far = math.hypot(x - px, y - py) >= self.min_dist_from_place
        return self.r[0] <= rad <= self.r[1] and far

    def sample(self, rng: random.Random,
               place_xy: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        """x, y 範囲から一様サンプルし、半径範囲・置き場所からの距離制約を満たすまで棄却する."""
        for _ in range(self.max_attempts):
            x = rng.uniform(*self.x)
            y = rng.uniform(*self.y)
            if self.is_valid(x, y, place_xy):
                return (x, y)
        raise SamplingError(f'no valid sample in {self.max_attempts} attempts')


@dataclass(frozen=True)
class PlaceSampler:
    """置き場所の中心位置のサンプラ. x, y 範囲から一様サンプルし、半径範囲（IK で置ける範囲）で棄却する."""
    x: Range
    y: Range
    r: Range
    max_attempts: int = 1000

    @classmethod
    def from_dict(cls, cfg: Mapping) -> 'PlaceSampler':
        def rng(k):
            return (float(cfg[k][0]), float(cfg[k][1]))
        return cls(x=rng('x'), y=rng('y'), r=rng('r'), max_attempts=int(cfg.get('max_attempts', 1000)))

    def sample(self, rng: random.Random) -> Tuple[float, float]:
        for _ in range(self.max_attempts):
            x = rng.uniform(*self.x)
            y = rng.uniform(*self.y)
            if self.r[0] <= math.hypot(x, y) <= self.r[1]:
                return (x, y)
        raise SamplingError(f'no valid place sample in {self.max_attempts} attempts')


def sample_scene(sampler: ObjectSampler, place_sampler: Optional[PlaceSampler],
                 rng: random.Random) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """(物体位置, 置き場所) を返す. place_sampler が None なら置き場所は設定の固定位置.

    置き場所を先に決め、物体はそこから min_dist_from_place 以上離れた位置に置く。
    """
    place = place_sampler.sample(rng) if place_sampler is not None else sampler.place_xy
    return sampler.sample(rng, place), place


def sample_positions(sampler: ObjectSampler, n: int, seed: int,
                     place_sampler: Optional[PlaceSampler] = None):
    """評価用に、同じ条件で再現可能な位置リストを生成する.

    置き場所を固定する場合は物体位置 (x, y)、ランダム化する場合は (物体 x, y, 置き場所 x, y) のリスト。
    """
    rng = random.Random(seed)
    if place_sampler is None:
        return [sampler.sample(rng) for _ in range(n)]
    return [(*o, *p) for o, p in (sample_scene(sampler, place_sampler, rng) for _ in range(n))]
