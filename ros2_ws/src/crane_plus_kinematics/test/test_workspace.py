import pytest

from crane_plus_kinematics.workspace import Workspace


@pytest.fixture
def ws():
    return Workspace(x=(0.05, 0.25), y=(-0.15, 0.15), z=(0.0, 0.25), r=(0.08, 0.26))


def test_inside_point_is_unchanged(ws):
    assert ws.clip(0.15, 0.0, 0.1) == pytest.approx((0.15, 0.0, 0.1))
    assert ws.contains(0.15, 0.0, 0.1)


def test_box_clip(ws):
    assert ws.clip(0.15, 0.0, -0.05) == pytest.approx((0.15, 0.0, 0.0))
    assert not ws.contains(0.15, 0.0, -0.05)


def test_radial_max_clip(ws):
    x, y, z = ws.clip(0.25, 0.15, 0.1)
    assert (x ** 2 + y ** 2) ** 0.5 == pytest.approx(0.26)


def test_radial_min_clip(ws):
    x, y, _ = ws.clip(0.05, 0.0, 0.1)
    assert (x ** 2 + y ** 2) ** 0.5 == pytest.approx(0.08)


def test_from_dict():
    ws = Workspace.from_dict({'x': [0, 1], 'y': [-1, 1], 'z': [0, 1], 'r': [0, 2]})
    assert ws.contains(0.5, 0.0, 0.5)
