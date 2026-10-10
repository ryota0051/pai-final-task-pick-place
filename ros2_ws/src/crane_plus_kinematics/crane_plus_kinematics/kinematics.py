"""CRANE+ V2 の順運動学・解析的逆運動学（ROS 非依存）.

アームはヨー軸 (joint1, z 軸) と、アーム平面内で回転する 3 つのピッチ軸
(joint2: +y 軸, joint3/joint4: -y 軸) からなる。
アーム平面内の角度は「鉛直上向きから前方 (+r) への傾き」phi で表す。

    phi2 = q2, phi3 = q2 - q3, phi4 = q2 - q3 - q4

手先姿勢の pitch は「水平から下向きを正」とした手先 (link4 z 軸) の角度で、
pitch = phi4 - pi/2 （真下向き = +pi/2, 水平前向き = 0, 真上向き = -pi/2）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

import yaml

ARM_JOINT_NAMES = (
    'crane_plus_joint1',
    'crane_plus_joint2',
    'crane_plus_joint3',
    'crane_plus_joint4',
)

_EPS = 1e-9


@dataclass(frozen=True)
class KinematicParams:
    base_to_joint1: float
    joint1_to_joint2: float
    l2: float
    l3: float
    tool_length: float
    tool_offset_y: float = 0.0

    @property
    def shoulder_height(self) -> float:
        return self.base_to_joint1 + self.joint1_to_joint2


@dataclass(frozen=True)
class EndEffectorPose:
    """crane_plus_base 座標系での把持中心位置 [m] と pitch [rad]."""

    x: float
    y: float
    z: float
    pitch: float


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ArmKinematics:
    def __init__(self, params: KinematicParams, limits: Mapping[str, Sequence[float]]):
        self.params = params
        self.limits = {k: (float(v[0]), float(v[1])) for k, v in limits.items()}

    # ---- 順運動学 ----
    def _plane_angles(self, q: Sequence[float]):
        phi2 = q[1]
        phi3 = q[1] - q[2]
        phi4 = q[1] - q[2] - q[3]
        return phi2, phi3, phi4

    def _to_base(self, q1: float, r: float, h: float, lateral: float = 0.0):
        c, s = math.cos(q1), math.sin(q1)
        return (r * c - lateral * s, r * s + lateral * c, h)

    def forward(self, q: Sequence[float]) -> EndEffectorPose:
        p = self.params
        phi2, phi3, phi4 = self._plane_angles(q)
        r = p.l2 * math.sin(phi2) + p.l3 * math.sin(phi3) + p.tool_length * math.sin(phi4)
        h = (p.shoulder_height + p.l2 * math.cos(phi2) + p.l3 * math.cos(phi3)
             + p.tool_length * math.cos(phi4))
        x, y, z = self._to_base(q[0], r, h, p.tool_offset_y)
        return EndEffectorPose(x=x, y=y, z=z, pitch=_wrap(phi4 - math.pi / 2))

    def elbow_position(self, q: Sequence[float]):
        """joint3 の位置."""
        p = self.params
        phi2, _, _ = self._plane_angles(q)
        return self._to_base(q[0], p.l2 * math.sin(phi2), p.shoulder_height + p.l2 * math.cos(phi2))

    def wrist_position(self, q: Sequence[float]):
        """joint4 の位置."""
        p = self.params
        phi2, phi3, _ = self._plane_angles(q)
        r = p.l2 * math.sin(phi2) + p.l3 * math.sin(phi3)
        h = p.shoulder_height + p.l2 * math.cos(phi2) + p.l3 * math.cos(phi3)
        return self._to_base(q[0], r, h)

    # ---- 関節リミット ----
    def within_limits(self, q: Sequence[float]) -> bool:
        for name, value in zip(ARM_JOINT_NAMES, q):
            lo, hi = self.limits[name]
            if value < lo - _EPS or value > hi + _EPS:
                return False
        return True

    # ---- 逆運動学 ----
    def inverse(self, target: EndEffectorPose) -> Optional[list]:
        """解析的 IK。関節リミット内の解のうち肘上 (肘が高い) 解を返す。解なしは None."""
        p = self.params
        rho2 = target.x ** 2 + target.y ** 2
        d = p.tool_offset_y
        if rho2 < d * d - _EPS:
            return None
        r_abs = math.sqrt(max(rho2 - d * d, 0.0))
        phi4 = target.pitch + math.pi / 2

        candidates = []
        for r in (r_abs, -r_abs):
            q1 = _wrap(math.atan2(target.y, target.x) - math.atan2(d, r))
            # リスト (joint4) 位置: 肩基準のアーム平面座標 (h: 鉛直, r: 前方)
            wr = r - p.tool_length * math.sin(phi4)
            wh = target.z - p.shoulder_height - p.tool_length * math.cos(phi4)
            cos_b = (wr ** 2 + wh ** 2 - p.l2 ** 2 - p.l3 ** 2) / (2 * p.l2 * p.l3)
            if cos_b > 1 + 1e-9 or cos_b < -1 - 1e-9:
                continue
            cos_b = min(1.0, max(-1.0, cos_b))
            for beta in (math.acos(cos_b), -math.acos(cos_b)):
                phi2 = math.atan2(wr, wh) - math.atan2(p.l3 * math.sin(beta),
                                                       p.l2 + p.l3 * math.cos(beta))
                phi3 = phi2 + beta
                q = [q1, _wrap(phi2), _wrap(phi2 - phi3), _wrap(phi3 - phi4)]
                if self.within_limits(q):
                    candidates.append(q)

        if not candidates:
            return None
        return max(candidates, key=lambda q: self.elbow_position(q)[2])


def load_kinematics(path: str | Path) -> ArmKinematics:
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    return kinematics_from_dict(cfg)


def kinematics_from_dict(cfg: Mapping) -> ArmKinematics:
    links = cfg['links']
    params = KinematicParams(
        base_to_joint1=float(links['base_to_joint1']),
        joint1_to_joint2=float(links['joint1_to_joint2']),
        l2=float(links['joint2_to_joint3']),
        l3=float(links['joint3_to_joint4']),
        tool_length=float(links['tool_length']),
        tool_offset_y=float(links.get('tool_offset_y', 0.0)),
    )
    return ArmKinematics(params, cfg['joint_limits'])
