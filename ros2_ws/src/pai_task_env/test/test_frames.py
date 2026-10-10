import pytest

from pai_task_env.frames import BaseFrame


def test_world_to_base_and_back():
    f = BaseFrame(origin=(0.0, 0.0, 1.015), yaw=0.0)
    assert f.world_to_base((0.2, 0.1, 1.03)) == pytest.approx((0.2, 0.1, 0.015))
    assert f.base_to_world((0.2, 0.1, 0.015)) == pytest.approx((0.2, 0.1, 1.03))


def test_yaw_rotation():
    f = BaseFrame(origin=(1.0, 0.0, 0.0), yaw=1.5707963267948966)
    # base の +x は world の +y
    assert f.base_to_world((0.1, 0.0, 0.0)) == pytest.approx((1.0, 0.1, 0.0))
    assert f.world_to_base((1.0, 0.1, 0.0)) == pytest.approx((0.1, 0.0, 0.0))
