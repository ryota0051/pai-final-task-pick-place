import math
from pathlib import Path

import pytest

from crane_plus_kinematics import Workspace, load_kinematics
from pai_web_teleop.teleop_state import EETeleopState, gripper_to_angle

KIN_CFG = Path(__file__).resolve().parents[2] / 'crane_plus_kinematics' / 'config' / 'kinematics.yaml'


@pytest.fixture
def state():
    kin = load_kinematics(KIN_CFG)
    ws = Workspace(x=(0.0, 0.26), y=(-0.2, 0.2), z=(0.0, 0.3), r=(0.06, 0.26))
    return EETeleopState(kin, ws, home_joints=[0.0, 0.6, -1.4, -1.2],
                         pitch_range=(-0.5, math.pi / 2))


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
    # 作業空間内だが真上向き pitch では届かない姿勢
    ok = state.set_target(0.26, 0.0, 0.0, -0.5)
    assert not ok
    assert state.joints == before_joints
    assert state.target == before_target
    assert state.last_ik_ok is False


def test_apply_velocity_integrates(state):
    state.set_target(0.15, 0.0, 0.05, math.pi / 2)
    state.apply_velocity(vx=0.1, vy=0.0, vz=0.0, vpitch=0.0, dt=0.1)
    assert state.target.x == pytest.approx(0.16)


def test_reset_home(state):
    state.set_target(0.15, 0.0, 0.05, math.pi / 2)
    state.reset_home()
    assert state.joints == pytest.approx([0.0, 0.6, -1.4, -1.2])


def test_gripper_to_angle():
    assert gripper_to_angle(0.0, open_angle=-0.5236, close_angle=0.1745) == pytest.approx(-0.5236)
    assert gripper_to_angle(1.0, open_angle=-0.5236, close_angle=0.1745) == pytest.approx(0.1745)
    assert gripper_to_angle(2.0, open_angle=-0.5, close_angle=0.5) == pytest.approx(0.5)
    assert gripper_to_angle(0.5, open_angle=-0.5, close_angle=0.5) == pytest.approx(0.0)
