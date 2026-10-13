import math
import random

import pytest

from pai_task_env.scripted_policy import PickPlacePlan, PlanParams, Waypoint, random_start_offset


@pytest.fixture
def params():
    return PlanParams(z_approach=0.08, z_grasp=0.02, z_carry=0.08, z_place=0.025, pitch=math.pi / 2,
                      move_speed=0.05, min_segment_time=0.5, grasp_hold=0.8, release_hold=0.6)


@pytest.fixture
def plan(params):
    return PickPlacePlan.create(start=(0.10, 0.0, 0.08), obj_xy=(0.15, 0.05),
                                place_xy=(0.10, -0.12), params=params)


def test_waypoint_sequence(plan):
    phases = [w.phase for w in plan.waypoints]
    assert phases == ['start', 'approach', 'descend', 'grasp', 'lift', 'carry', 'lower', 'release', 'retreat']


def test_gripper_state_per_phase(plan):
    g = {w.phase: w.gripper for w in plan.waypoints}
    assert g['approach'] == 0.0 and g['descend'] == 0.0
    assert g['grasp'] == 1.0 and g['lift'] == 1.0 and g['carry'] == 1.0 and g['lower'] == 1.0
    assert g['release'] == 0.0 and g['retreat'] == 0.0


def test_positions(plan):
    w = {w.phase: w for w in plan.waypoints}
    assert (w['descend'].x, w['descend'].y, w['descend'].z) == pytest.approx((0.15, 0.05, 0.02))
    assert (w['lower'].x, w['lower'].y, w['lower'].z) == pytest.approx((0.10, -0.12, 0.025))


def test_segment_time_from_distance(plan, params):
    w = {w.phase: w for w in plan.waypoints}
    # approach: (0.10,0,0.08) -> (0.15,0.05,0.08) 距離 0.0707 / 0.05 = 1.414s
    assert w['approach'].duration == pytest.approx(math.hypot(0.05, 0.05) / 0.05)
    # descend: 0.06 / 0.05 = 1.2s
    assert w['descend'].duration == pytest.approx(1.2)
    # 短い区間は min_segment_time
    assert w['grasp'].duration == pytest.approx(params.grasp_hold)


def test_sample_interpolates(plan):
    t0 = plan.start_time_of('descend')
    mid = plan.sample(t0 + plan.duration_of('descend') / 2)
    assert mid.z == pytest.approx(0.05, abs=0.005)
    assert mid.gripper == 0.0


def test_sample_clamps_after_end(plan):
    end = plan.sample(plan.total_duration + 10.0)
    last = plan.waypoints[-1]
    assert (end.x, end.y, end.z) == pytest.approx((last.x, last.y, last.z))


def test_sample_is_smooth_at_boundaries(plan):
    # min-jerk 補間なので区間境界で位置が連続
    t = plan.start_time_of('lift')
    a, b = plan.sample(t - 1e-6), plan.sample(t + 1e-6)
    assert (a.x, a.y, a.z) == pytest.approx((b.x, b.y, b.z), abs=1e-4)


def test_randomized_params_within_bounds(params):
    rng = random.Random(0)
    for _ in range(50):
        p = params.randomized(rng, speed_scale=(0.8, 1.2), height_jitter=0.01)
        assert 0.04 <= p.move_speed <= 0.06
        assert abs(p.z_approach - 0.08) <= 0.01 and abs(p.z_carry - 0.08) <= 0.01
        assert p.z_grasp == params.z_grasp


def test_waypoint_is_dataclass():
    w = Waypoint('x', 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert w.phase == 'x'


def test_random_start_offset_bounded_and_reproducible():
    a = [random_start_offset(random.Random(5), (0.03, 0.03, 0.02)) for _ in range(3)]
    b = [random_start_offset(random.Random(5), (0.03, 0.03, 0.02)) for _ in range(3)]
    assert a == b
    rng = random.Random(0)
    for _ in range(100):
        dx, dy, dz = random_start_offset(rng, (0.03, 0.03, 0.02))
        assert abs(dx) <= 0.03 and abs(dy) <= 0.03 and abs(dz) <= 0.02


def test_random_start_offset_zero_does_not_consume_rng():
    rng = random.Random(1)
    assert random_start_offset(rng, (0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    # ばらつきなしの設定では既存のデモ生成と乱数列が変わらない（seed の再現性を保つ）
    assert rng.random() == random.Random(1).random()


def test_grasp_settle_waits_open_at_bottom_before_closing(params):
    """下降後、グリッパを開いたまま grasp_settle 秒待ってから閉じる（腕の追従遅れで下降中に閉じると物体を押すため）."""
    from dataclasses import replace
    p = replace(params, grasp_settle=0.6)
    plan = PickPlacePlan.create(start=(0.10, 0.0, 0.08), obj_xy=(0.15, 0.05), place_xy=(0.10, -0.12), params=p)
    phases = [w.phase for w in plan.waypoints]
    assert phases[:5] == ['start', 'approach', 'descend', 'settle', 'grasp']
    settle = plan.waypoints[3]
    assert settle.gripper == 0.0 and settle.duration == pytest.approx(0.6)
    assert (settle.x, settle.y, settle.z) == (0.15, 0.05, 0.02)


def test_grasp_settle_zero_keeps_original_plan(plan):
    assert 'settle' not in [w.phase for w in plan.waypoints]
