"""カメラ画像からの物体位置推定（ROS 非依存, numpy のみ）.

カメラ座標系は Gazebo のカメラリンクの規約（+x が光軸、+y が左、+z が上）。
画像座標は u が右、v が下なので、u ↔ -y、v ↔ -z に対応する。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll)（URDF / SDF と同じ規約）."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    position: Sequence[float]     # crane_plus_base 基準のカメラ位置 [m]
    rpy: Sequence[float]          # crane_plus_base 基準のカメラ姿勢 [rad]

    def __post_init__(self):
        self.R = rpy_to_matrix(*self.rpy)
        self.t = np.asarray(self.position, dtype=float)

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov: float, position, rpy) -> 'CameraModel':
        """Gazebo の camera センサ（正方画素）の内部パラメータを水平画角から求める."""
        f = (width / 2.0) / math.tan(hfov / 2.0)
        return cls(fx=f, fy=f, cx=width / 2.0, cy=height / 2.0, position=position, rpy=rpy)

    def project(self, p: Sequence[float]) -> Tuple[float, float]:
        pc = self.R.T @ (np.asarray(p, dtype=float) - self.t)
        return (self.cx - self.fx * pc[1] / pc[0], self.cy - self.fy * pc[2] / pc[0])

    def pixel_to_plane(self, u: float, v: float, plane_z: float) -> Tuple[float, float, float]:
        """画素 (u, v) を通る視線と水平面 z = plane_z の交点."""
        d = self.R @ np.array([1.0, -(u - self.cx) / self.fx, -(v - self.cy) / self.fy])
        if abs(d[2]) < 1e-12:
            raise ValueError('ray is parallel to the plane')
        s = (plane_z - self.t[2]) / d[2]
        p = self.t + s * d
        return (float(p[0]), float(p[1]), float(p[2]))


def red_mask(rgb: np.ndarray, min_red: int, max_other_ratio: float) -> np.ndarray:
    """赤い画素のマスク: R >= min_red かつ G, B <= R * max_other_ratio."""
    r = rgb[..., 0].astype(np.float32)
    g = rgb[..., 1].astype(np.float32)
    b = rgb[..., 2].astype(np.float32)
    return (r >= min_red) & (g <= r * max_other_ratio) & (b <= r * max_other_ratio)


def mask_centroid(mask: np.ndarray, min_pixels: int) -> Optional[Tuple[float, float]]:
    """マスクの重心 (u, v)。画素数が min_pixels 未満なら None."""
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return None
    return (float(us.mean()), float(vs.mean()))


class EstimateHolder:
    """物体位置推定値の保持.

    遮蔽（腕が物体を隠す）で検出できない間は最後の推定値を返す（hold_last）。
    腕が物体の一部だけを隠すと、見えている部分の重心がずれて推定値が物体から離れる方向へ動く。
    そこで見えている画素数がエピソード内の最大値の min_visible_ratio 未満の検出は部分遮蔽とみなして採用しない。
    タスクのエピソードが変わったら前エピソードの値は破棄し、新たに検出されるまで None を返す。
    """

    def __init__(self, hold_last: bool, min_visible_ratio: float = 0.0):
        self.hold_last = hold_last
        self.min_visible_ratio = min_visible_ratio
        self.last: Optional[Tuple[float, float, float]] = None
        self.episode = None
        self.detected_in_episode = False
        self.max_pixels = 0
        self.partially_occluded = False

    def update(self, detection, episode, pixels: Optional[int] = None) -> Optional[Tuple[float, float, float]]:
        if episode is not None and episode != self.episode:
            self.episode = episode
            self.last = None
            self.detected_in_episode = False
            self.max_pixels = 0
        self.partially_occluded = False
        if detection is not None and pixels is not None:
            self.max_pixels = max(self.max_pixels, pixels)
            if pixels < self.min_visible_ratio * self.max_pixels:
                self.partially_occluded = True
                detection = None
        if detection is not None:
            self.last = tuple(detection)
            self.detected_in_episode = True
            return self.last
        return self.last if self.hold_last else None
