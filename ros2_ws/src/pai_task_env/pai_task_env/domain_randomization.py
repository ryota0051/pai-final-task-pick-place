"""物体検出器の学習データ用のドメインランダム化と正解ラベル（ROS 非依存）.

物体の色・ダミー物体・机の色味・照明をランダムに変えた俯瞰カメラ画像を作り、
正解の囲み枠は物体の真の位置と円柱の形状をカメラモデルで画像へ投影して付ける（ADR 0008）。
"""
from __future__ import annotations

import colorsys
import math
import random
from dataclasses import dataclass, field
from typing import List, Mapping, Optional, Sequence, Tuple

import numpy as np

from pai_task_env.vision import CameraModel


def cylinder_bbox(cam: CameraModel, center: Sequence[float], radius: float, height: float,
                  image_size: Tuple[int, int], n: int = 32) -> Tuple[float, float, float, float]:
    """円柱（軸は鉛直）の上面・底面の円周を投影した点の外接矩形 (u0, v0, u1, v1). 画像内にクリップする."""
    cx, cy, cz = center
    pts = []
    for z in (cz - height / 2, cz + height / 2):
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append(cam.project((cx + radius * math.cos(a), cy + radius * math.sin(a), z)))
    us, vs = zip(*pts)
    w, h = image_size
    return (max(0.0, min(us)), max(0.0, min(vs)), min(float(w), max(us)), min(float(h), max(vs)))


def hsv_to_rgb(h: float, s: float, v: float) -> Tuple[float, float, float]:
    return colorsys.hsv_to_rgb(h, s, v)


def polyline_clear_of(points: Sequence[Sequence[float]], xy: Sequence[float], min_dist: float) -> bool:
    """折れ線（腕の関節位置を結んだもの）が、点 xy から水平距離 min_dist 以上離れているか."""
    p = np.asarray(xy, dtype=float)
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a[:2], dtype=float), np.asarray(b[:2], dtype=float)
        ab = b - a
        t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
        if np.linalg.norm(a + t * ab - p) < min_dist:
            return False
    return True


@dataclass(frozen=True)
class Distractor:
    xy: Tuple[float, float]
    size: float                        # 立方体の一辺 [m]
    yaw: float
    rgb: Tuple[float, float, float]


@dataclass(frozen=True)
class SceneParams:
    object_rgb: Tuple[float, float, float]
    table_tint: Tuple[float, float, float]
    light_intensity: float
    light_direction: Tuple[float, float, float]
    distractors: Tuple[Distractor, ...] = field(default_factory=tuple)
    place_rgb: Optional[Tuple[float, float, float]] = None   # 置き場所の色（設定に place があるときだけ変える）


class SceneSampler:
    def __init__(self, cfg: Mapping):
        self.cfg = cfg

    def _color(self, rng: random.Random, key: str = 'object') -> Tuple[float, float, float]:
        o = self.cfg[key]
        return hsv_to_rgb(rng.random(), rng.uniform(*o['saturation']), rng.uniform(*o['value']))

    def sample(self, rng: random.Random, object_xy: Sequence[float],
               place_xy: Optional[Sequence[float]] = None) -> SceneParams:
        """place_xy を渡すと、ダミーの箱を置き場所から min_dist_place（既定は min_dist）以上離す."""
        d = self.cfg['distractors']
        min_dist_place = d.get('min_dist_place', d['min_dist'])
        placed: List[Distractor] = []
        for _ in range(rng.randint(*d['count'])):
            for _try in range(100):
                xy = (rng.uniform(*d['x']), rng.uniform(*d['y']))
                if math.dist(xy, object_xy) < d['min_dist']:
                    continue
                if place_xy is not None and math.dist(xy, place_xy) < min_dist_place:
                    continue
                if any(math.dist(xy, q.xy) < d['min_dist'] for q in placed):
                    continue
                placed.append(Distractor(xy, rng.uniform(*d['size']), rng.uniform(0, math.pi), self._color(rng)))
                break
        tilt = rng.uniform(0.0, self.cfg['light']['max_tilt'])
        az = rng.uniform(-math.pi, math.pi)
        direction = (math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt))
        # 置き場所の色は最後に引く（設定に place がない場合と、ほかの値の乱数列を変えないため）
        params = dict(
            object_rgb=self._color(rng),
            table_tint=tuple(rng.uniform(*self.cfg['table_tint']) for _ in range(3)),
            light_intensity=rng.uniform(*self.cfg['light']['intensity']),
            light_direction=direction,
            distractors=tuple(placed),
        )
        if 'place' in self.cfg:
            params['place_rgb'] = self._color(rng, 'place')
        return SceneParams(**params)


def bbox_center_to_position(cam: CameraModel, center_uv: Sequence[float], radius: float, height: float,
                            image_size: Tuple[int, int], center_z: float, iterations: int = 10) -> Tuple[float, float]:
    """検出した囲み枠の中心 (u, v) から円柱の中心位置 (x, y) を逆算する.

    枠の中心は側面の見え方の分だけ物体中心の投影からずれるので、
    「その位置に円柱があるときの枠の中心」が検出値に一致するよう不動点反復で補正する。
    """
    u, v = center_uv
    target = np.asarray(cam.pixel_to_plane(u, v, center_z)[:2])
    p = target.copy()
    for _ in range(iterations):
        u0, v0, u1, v1 = cylinder_bbox(cam, (p[0], p[1], center_z), radius, height, image_size)
        pred = np.asarray(cam.pixel_to_plane((u0 + u1) / 2, (v0 + v1) / 2, center_z)[:2])
        p = p + (target - pred)
    return float(p[0]), float(p[1])
