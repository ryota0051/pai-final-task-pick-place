import math
from pathlib import Path

import pytest

from crane_plus_kinematics import Workspace, load_kinematics
from crane_plus_kinematics import EndEffectorPose
from pai_web_teleop.teleop_state import EETeleopState, gripper_to_angle

KIN_CFG = (Path(__file__).resolve().parents[2] / 'crane_plus_kinematics' / 'test' / 'fixtures'
           / 'kinematics_test.yaml')


@pytest.fixture
def state():
    kin = load_kinematics(KIN_CFG)
    ws = Workspace(x=(0.0, 0.26), y=(-0.2, 0.2), z=(0.0, 0.3), r=(0.06, 0.26))
    return EETeleopState(kin, ws, home_joints=[0.0, 0.6, -1.4, -1.2],
                         pitch_range=(-0.5, math.pi / 2), pitch_search_step=0.05)


def test_initial_target_is_fk_of_home(state):
    fk = state.kin.forward([0.0, 0.6, -1.4, -1.2])
    t = state.target
    assert (t.x, t.y, t.z, t.pitch) == pytest.approx((fk.x, fk.y, fk.z, fk.pitch))
    assert state.joints == pytest.approx([0.0, 0.6, -1.4, -1.2])


def test_set_target_updates_joints(state):
    ok = state.set_target(0.15, 0.03, 0.05, math.pi / 2)
    assert ok
    fk = state.kin.forward(state.joints)
    assert (fk.x, fk.y, fk.z) == pytest.approx((0.15, 0.03, 0.05), abs=1e-6)


def test_set_target_is_clipped_to_workspace(state):
    state.set_target(0.15, 0.0, -0.10, math.pi / 2)
    assert state.target.z == pytest.approx(0.0)


def test_pitch_is_clipped(state):
    state.set_target(0.15, 0.0, 0.05, 3.0)
    assert state.target.pitch == pytest.approx(math.pi / 2)


def test_unreachable_target_keeps_previous(state):
    state.set_target(0.15, 0.0, 0.05, math.pi / 2)
    before_joints = list(state.joints)
    before_target = state.target
    # 作業空間内だがどの pitch でも届かない位置
    ok = state.set_target(0.26, 0.0, 0.25, 0.0)
    assert not ok
    assert state.joints == before_joints
    assert state.target == before_target
    assert state.last_ik_ok is False


def test_apply_velocity_integrates(state):
    state.set_target(0.12, 0.0, 0.05, math.pi / 2)
    state.apply_velocity(vx=0.1, vy=0.0, vz=0.0, vpitch=0.0, dt=0.1)
    assert state.target.x == pytest.approx(0.13)


def test_pitch_is_relaxed_when_position_unreachable(state):
    # 真下向きでは届かない位置でも、要求に最も近い解ける pitch で位置を優先する
    assert state.kin.inverse(EndEffectorPose(0.19, 0.0, 0.05, math.pi / 2)) is None
    ok = state.set_target(0.19, 0.0, 0.05, math.pi / 2)
    assert ok
    assert (state.target.x, state.target.z) == pytest.approx((0.19, 0.05))
    assert state.target.pitch < math.pi / 2
    assert state.target.pitch > math.pi / 2 - 0.6
    assert state.pitch_relaxed


def test_requested_pitch_is_kept_for_velocity_commands(state):
    # 位置の都合で pitch が緩和されても、戻れば要求 pitch に復帰する
    state.set_target(0.19, 0.0, 0.05, math.pi / 2)
    state.apply_velocity(vx=-0.7, vy=0.0, vz=0.0, vpitch=0.0, dt=0.1)
    assert state.target.x == pytest.approx(0.12)
    assert state.target.pitch == pytest.approx(math.pi / 2)
    assert not state.pitch_relaxed


def test_reset_home(state):
    state.set_target(0.15, 0.0, 0.05, math.pi / 2)
    state.reset_home()
    assert state.joints == pytest.approx([0.0, 0.6, -1.4, -1.2])


def test_gripper_to_angle():
    assert gripper_to_angle(0.0, open_angle=-0.5236, close_angle=0.1745) == pytest.approx(-0.5236)
    assert gripper_to_angle(1.0, open_angle=-0.5236, close_angle=0.1745) == pytest.approx(0.1745)
    assert gripper_to_angle(2.0, open_angle=-0.5, close_angle=0.5) == pytest.approx(0.5)
    assert gripper_to_angle(0.5, open_angle=-0.5, close_angle=0.5) == pytest.approx(0.0)


# ---- 外乱（DART）: 目標からずらした位置の関節角を、状態を変えずに求める ----
def test_joints_with_offset_moves_ee_without_changing_state(state):
    assert state.set_target(0.15, 0.03, 0.05, 1.134)
    before_joints, before_target = list(state.joints), state.target
    q = state.joints_with_offset(0.005, -0.004, 0.002)
    fk = state.kin.forward(q)
    assert (fk.x, fk.y, fk.z) == pytest.approx((0.155, 0.026, 0.052), abs=1e-6)
    assert fk.pitch == pytest.approx(1.134)
    # 目標（＝ラベルにする外乱なしの指令）は変わらない
    assert state.joints == before_joints
    assert state.target == before_target


def test_joints_with_zero_offset_is_current_joints(state):
    state.set_target(0.15, 0.03, 0.05, 1.134)
    assert state.joints_with_offset(0.0, 0.0, 0.0) == state.joints


def test_joints_with_offset_is_clipped_to_workspace(state):
    state.set_target(0.15, 0.0, 0.002, math.pi / 2)
    fk = state.kin.forward(state.joints_with_offset(0.0, 0.0, -0.01))
    assert fk.z == pytest.approx(0.0, abs=1e-6)


def test_joints_with_unreachable_offset_falls_back_to_current(state):
    state.set_target(0.15, 0.0, 0.05, math.pi / 2)
    assert state.joints_with_offset(0.5, 0.0, 0.5) == state.joints
