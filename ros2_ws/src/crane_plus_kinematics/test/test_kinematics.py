import math
import random
from pathlib import Path

import pytest

from crane_plus_kinematics.kinematics import (
    ArmKinematics,
    EndEffectorPose,
    load_kinematics,
)

CONFIG = Path(__file__).resolve().parent / 'fixtures' / 'kinematics_test.yaml'
REAL_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'kinematics.yaml'


@pytest.fixture(scope='module')
def kin() -> ArmKinematics:
    return load_kinematics(CONFIG)


def test_load_config_values(kin):
    assert kin.params.l2 == pytest.approx(0.083)
    assert kin.params.l3 == pytest.approx(0.0935)
    assert kin.params.shoulder_height == pytest.approx(0.0454 + 0.026)
    assert kin.limits['crane_plus_joint1'] == pytest.approx((-2.61799, 2.61799))


def test_real_config_is_loadable_and_consistent(kin):
    real = load_kinematics(REAL_CONFIG)
    # リンク長・リミットは URDF 由来で固定、ツール長のみ調整対象
    assert real.params.l2 == kin.params.l2 and real.params.l3 == kin.params.l3
    assert real.limits == kin.limits
    assert 0.05 < real.params.tool_length < 0.15
    assert abs(real.params.tool_offset_y) < 0.04


def test_fk_with_tool_offset_y():
    import dataclasses
    base = load_kinematics(CONFIG)
    params = dataclasses.replace(base.params, tool_offset_y=0.02)
    from crane_plus_kinematics.kinematics import ArmKinematics
    k = ArmKinematics(params, base.limits)
    p = k.forward([math.pi / 2, math.pi / 2, 0.0, 0.0])
    # yaw 90deg でアーム平面の前方は +y、オフセット方向は -x
    assert p.x == pytest.approx(-0.02)
    sol = k.inverse(p)
    assert sol is not None
    got = k.forward(sol)
    assert (got.x, got.y, got.z) == pytest.approx((p.x, p.y, p.z), abs=1e-6)


def test_fk_all_zero_points_straight_up(kin):
    p = kin.forward([0.0, 0.0, 0.0, 0.0])
    top = kin.params.shoulder_height + kin.params.l2 + kin.params.l3 + kin.params.tool_length
    assert p.x == pytest.approx(0.0, abs=1e-9)
    assert p.y == pytest.approx(kin.params.tool_offset_y, abs=1e-9)
    assert p.z == pytest.approx(top)
    # 真上向きは pitch = -90deg（水平より上向きが負）
    assert p.pitch == pytest.approx(-math.pi / 2)


def test_fk_joint2_positive_bows_forward(kin):
    # joint2 は +y 軸回り: 正方向で +x 側へ倒れる（演習の「お辞儀」）
    p = kin.forward([0.0, math.pi / 2, 0.0, 0.0])
    reach = kin.params.l2 + kin.params.l3 + kin.params.tool_length
    assert p.x == pytest.approx(reach)
    assert p.z == pytest.approx(kin.params.shoulder_height)
    assert p.pitch == pytest.approx(0.0, abs=1e-9)


def test_fk_joint3_positive_bends_backward(kin):
    # joint3 は -y 軸回り: 正方向で -x 側へ倒れる
    p = kin.forward([0.0, 0.0, math.pi / 2, 0.0])
    assert p.x == pytest.approx(-(kin.params.l3 + kin.params.tool_length))
    assert p.z == pytest.approx(kin.params.shoulder_height + kin.params.l2)


def test_fk_joint1_rotates_about_z(kin):
    p0 = kin.forward([0.0, 0.6, -1.2, -0.8])
    p1 = kin.forward([math.pi / 2, 0.6, -1.2, -0.8])
    assert p1.x == pytest.approx(-p0.y, abs=1e-9)
    assert p1.y == pytest.approx(p0.x, abs=1e-9)
    assert p1.z == pytest.approx(p0.z)


def _random_joints(kin, rng):
    names = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
    q = []
    for n in names:
        lo, hi = kin.limits[n]
        q.append(rng.uniform(lo * 0.9, hi * 0.9))
    return q


def test_ik_fk_roundtrip(kin):
    rng = random.Random(0)
    checked = 0
    for _ in range(2000):
        q = _random_joints(kin, rng)
        target = kin.forward(q)
        # 前方作業空間（アーム平面で手先が base より前）のみ対象
        if math.hypot(target.x, target.y) < 0.03:
            continue
        sol = kin.inverse(target)
        if sol is None:
            continue
        got = kin.forward(sol)
        assert got.x == pytest.approx(target.x, abs=1e-6)
        assert got.y == pytest.approx(target.y, abs=1e-6)
        assert got.z == pytest.approx(target.z, abs=1e-6)
        assert got.pitch == pytest.approx(target.pitch, abs=1e-6)
        assert kin.within_limits(sol)
        checked += 1
    assert checked > 500


def test_ik_typical_grasp_pose_is_solvable(kin):
    # テーブル上の物体を真上から掴む典型姿勢
    target = EndEffectorPose(x=0.15, y=0.05, z=0.03, pitch=math.pi / 2)
    sol = kin.inverse(target)
    assert sol is not None
    got = kin.forward(sol)
    assert (got.x, got.y, got.z) == pytest.approx((0.15, 0.05, 0.03), abs=1e-6)


def test_ik_prefers_elbow_up(kin):
    target = EndEffectorPose(x=0.15, y=0.0, z=0.05, pitch=math.pi / 2)
    sol = kin.inverse(target)
    assert sol is not None
    elbow = kin.elbow_position(sol)
    # 肘上解: 肘が肩とリスト(joint4)を結ぶ線より上にある
    wrist = kin.wrist_position(sol)
    sh = kin.params.shoulder_height
    line_z = sh + (wrist[2] - sh) * (math.hypot(elbow[0], elbow[1]) / math.hypot(wrist[0], wrist[1]))
    assert elbow[2] > line_z


def test_ik_unreachable_returns_none(kin):
    assert kin.inverse(EndEffectorPose(x=1.0, y=0.0, z=0.1, pitch=0.0)) is None


def test_within_limits_rejects_out_of_range(kin):
    assert kin.within_limits([0.0, 0.0, 0.0, 0.0])
    assert not kin.within_limits([3.0, 0.0, 0.0, 0.0])
