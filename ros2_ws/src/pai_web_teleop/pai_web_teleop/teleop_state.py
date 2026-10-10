"""手先目標の保持・作業空間クリップ・IK 変換を行う状態クラス（ROS 非依存）."""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

from crane_plus_kinematics import ArmKinematics, EndEffectorPose, Workspace


def gripper_to_angle(value: float, open_angle: float, close_angle: float) -> float:
    """0 (開) 〜 1 (閉) のグリッパ指令を関節角に線形変換する."""
    v = min(max(value, 0.0), 1.0)
    return open_angle + (close_angle - open_angle) * v


class EETeleopState:
    def __init__(self, kin: ArmKinematics, workspace: Workspace,
                 home_joints: Sequence[float], pitch_range: Tuple[float, float]):
        self.kin = kin
        self.workspace = workspace
        self.home_joints = list(home_joints)
        self.pitch_range = pitch_range
        self.last_ik_ok: Optional[bool] = None
        self.reset_home()

    def reset_home(self) -> None:
        self.joints = list(self.home_joints)
        self.target = self.kin.forward(self.joints)
        self.last_ik_ok = True

    def set_target(self, x: float, y: float, z: float, pitch: float) -> bool:
        """目標を設定。IK 解なしなら直前の目標・関節角を維持して False を返す."""
        cx, cy, cz = self.workspace.clip(x, y, z)
        cp = min(max(pitch, self.pitch_range[0]), self.pitch_range[1])
        candidate = EndEffectorPose(cx, cy, cz, cp)
        sol = self.kin.inverse(candidate)
        self.last_ik_ok = sol is not None
        if sol is None:
            return False
        self.target = candidate
        self.joints = sol
        return True

    def apply_velocity(self, vx: float, vy: float, vz: float, vpitch: float, dt: float) -> bool:
        t = self.target
        return self.set_target(t.x + vx * dt, t.y + vy * dt, t.z + vz * dt, t.pitch + vpitch * dt)
