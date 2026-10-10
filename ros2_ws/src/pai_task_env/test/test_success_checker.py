import pytest

from pai_task_env.success_checker import SuccessTracker

TABLE_Z = 0.0125  # 物体中心のテーブル静置高さ


@pytest.fixture
def tracker():
    return SuccessTracker(place_xy=(0.12, -0.12), place_radius=0.03, rest_z=TABLE_Z,
                          z_tolerance=0.01, lift_height=0.02, hold_time=0.5)


def test_initial_status(tracker):
    s = tracker.update(0.0, (0.15, 0.05, TABLE_Z))
    assert s == {'lifted': False, 'in_place_zone': False, 'success': False}


def test_lift_is_latched(tracker):
    tracker.update(0.0, (0.15, 0.05, TABLE_Z))
    tracker.update(0.1, (0.15, 0.05, TABLE_Z + 0.05))
    s = tracker.update(0.2, (0.15, 0.05, TABLE_Z))
    assert s['lifted'] is True


def test_success_requires_lift_and_hold(tracker):
    tracker.update(0.0, (0.15, 0.05, TABLE_Z))
    tracker.update(1.0, (0.13, -0.05, TABLE_Z + 0.05))       # 持ち上げ
    s = tracker.update(2.0, (0.125, -0.115, TABLE_Z))         # 置き場所に着地
    assert s['in_place_zone'] and not s['success']
    s = tracker.update(2.4, (0.125, -0.115, TABLE_Z))
    assert not s['success']
    s = tracker.update(2.6, (0.125, -0.115, TABLE_Z))
    assert s['success']


def test_no_success_without_lift(tracker):
    # 押して移動させただけでは成功にしない
    tracker.update(0.0, (0.125, -0.115, TABLE_Z))
    s = tracker.update(5.0, (0.125, -0.115, TABLE_Z))
    assert not s['success']


def test_leaving_zone_resets_hold(tracker):
    tracker.update(0.0, (0.15, 0.05, TABLE_Z + 0.05))
    tracker.update(1.0, (0.12, -0.12, TABLE_Z))
    tracker.update(1.3, (0.20, -0.12, TABLE_Z))               # ゾーン外へ
    s = tracker.update(1.6, (0.12, -0.12, TABLE_Z))
    assert not s['success']
    s = tracker.update(2.2, (0.12, -0.12, TABLE_Z))
    assert s['success']


def test_reset(tracker):
    tracker.update(0.0, (0.15, 0.05, TABLE_Z + 0.05))
    tracker.reset()
    assert tracker.update(1.0, (0.15, 0.05, TABLE_Z))['lifted'] is False


def test_reach_mode_success_by_ee_distance():
    t = SuccessTracker(place_xy=(0, 0), place_radius=0.03, rest_z=TABLE_Z, z_tolerance=0.01,
                       lift_height=0.02, hold_time=0.2, mode='reach', reach_threshold=0.02)
    s = t.update(0.0, (0.15, 0.05, TABLE_Z), ee_xyz=(0.15, 0.05, 0.05))
    assert not s['success']
    t.update(1.0, (0.15, 0.05, TABLE_Z), ee_xyz=(0.15, 0.05, 0.025))
    s = t.update(1.3, (0.15, 0.05, TABLE_Z), ee_xyz=(0.151, 0.05, 0.025))
    assert s['success']
