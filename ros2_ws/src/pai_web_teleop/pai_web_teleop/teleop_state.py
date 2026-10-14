"""手先目標の保持・作業空間クリップ・IK 変換を行う状態クラス（ROS 非依存）."""
from __future__ import annotations

from typing import Iterator, List, Optional, Sequence, Tuple

from crane_plus_kinematics import ArmKinematics, EndEffectorPose, Workspace


def gripper_to_angle(value: float, open_angle: float, close_angle: float) -> float:
    """0 (開) 〜 1 (閉) のグリッパ指令を関節角に線形変換する."""
    v = min(max(value, 0.0), 1.0)
    return open_angle + (close_angle - open_angle) * v


class EETeleopState:
    """手先目標 (x, y, z, pitch) を保持し、IK で関節角に変換する.

    要求 pitch で IK が解けない場合は、pitch_range 内で要求値に近い順に pitch を探索し
    位置を優先する（4 軸アームでは姿勢より位置の追従が操作上重要なため）。
    要求 pitch は別に保持するので、届く位置に戻れば要求 pitch に復帰する。
    """

    def __init__(self, kin: ArmKinematics, workspace: Workspace,
                 home_joints: Sequence[float], pitch_range: Tuple[float, float],
                 pitch_search_step: float = 0.05):
        self.kin = kin
        self.workspace = workspace
        self.home_joints = list(home_joints)
        self.pitch_range = pitch_range
        self.pitch_search_step = pitch_search_step
        self.last_ik_ok: Optional[bool] = None
        self.reset_home()

    @property
    def pitch_relaxed(self) -> bool:
        return abs(self.target.pitch - self.requested_pitch) > 1e-9

    def reset_home(self) -> None:
        self.joints = list(self.home_joints)
        self.target = self.kin.forward(self.joints)
        self.requested_pitch = self._clip_pitch(self.target.pitch)
        self.last_ik_ok = True

    def _clip_pitch(self, pitch: float) -> float:
        return min(max(pitch, self.pitch_range[0]), self.pitch_range[1])

    def _pitch_candidates(self, requested: float) -> Iterator[float]:
        yield requested
        if self.pitch_search_step <= 0:
            return
        lo, hi = self.pitch_range
        k = 1
        while True:
            down, up = requested - k * self.pitch_search_step, requested + k * self.pitch_search_step
            if down < lo and up > hi:
                return
            if down >= lo:
                yield down
            if up <= hi:
                yield up
            k += 1

    def _solve(self, x: float, y: float, z: float,
               requested_pitch: float) -> Optional[Tuple[EndEffectorPose, List[float]]]:
        cx, cy, cz = self.workspace.clip(x, y, z)
        for p in self._pitch_candidates(requested_pitch):
            candidate = EndEffectorPose(cx, cy, cz, p)
            sol = self.kin.inverse(candidate)
            if sol is not None:
                return candidate, sol
        return None

    def set_target(self, x: float, y: float, z: float, pitch: float) -> bool:
        """目標を設定。どの pitch でも IK 解なしなら直前の状態を維持して False を返す."""
        requested = self._clip_pitch(pitch)
        solved = self._solve(x, y, z, requested)
        if solved is None:
            self.last_ik_ok = False
            return False
        self.target, self.joints = solved
        self.requested_pitch = requested
        self.last_ik_ok = True
        return True

    def joints_with_offset(self, dx: float, dy: float, dz: float) -> List[float]:
        """目標から (dx, dy, dz) ずらした位置の関節角. 状態（目標・関節角）は変えない.

        DART のデモ収録用: 実行する指令にだけ外乱を足し、ラベルは外乱なしの self.joints にする。
        IK が解けなければ外乱なしの関節角を返す。
        """
        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return self.joints
        t = self.target
        solved = self._solve(t.x + dx, t.y + dy, t.z + dz, self.requested_pitch)
        return self.joints if solved is None else solved[1]

    def apply_velocity(self, vx: float, vy: float, vz: float, vpitch: float, dt: float) -> bool:
        t = self.target
        return self.set_target(t.x + vx * dt, t.y + vy * dt, t.z + vz * dt,
                               self.requested_pitch + vpitch * dt)
