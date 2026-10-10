import pytest

from pai_task_env.grasp_assist import GraspAssist


@pytest.fixture
def ga():
    return GraspAssist(close_threshold=0.12, open_threshold=0.0, attach_distance=0.015)


def test_attach_when_closed_and_near(ga):
    assert ga.update(hand_angle=0.19, grasp_center=(0.1, 0.0, 0.02), object_xyz=(0.105, 0.0, 0.02)) == 'attach'
    assert ga.attached


def test_no_attach_when_far(ga):
    assert ga.update(0.30, (0.1, 0.0, 0.02), (0.13, 0.0, 0.02)) is None
    assert not ga.attached


def test_no_attach_when_open(ga):
    assert ga.update(-0.5, (0.1, 0.0, 0.02), (0.1, 0.0, 0.02)) is None


def test_no_repeat_attach(ga):
    ga.update(0.19, (0.1, 0.0, 0.02), (0.1, 0.0, 0.02))
    assert ga.update(0.19, (0.1, 0.0, 0.05), (0.1, 0.0, 0.05)) is None


def test_detach_when_opened(ga):
    ga.update(0.19, (0.1, 0.0, 0.02), (0.1, 0.0, 0.02))
    # しきい値の間（ヒステリシス帯）では保持
    assert ga.update(0.05, (0.1, 0.0, 0.05), (0.1, 0.0, 0.05)) is None
    assert ga.update(-0.1, (0.1, 0.0, 0.05), (0.1, 0.0, 0.05)) == 'detach'
    assert not ga.attached


def test_force_detach(ga):
    ga.update(0.19, (0.1, 0.0, 0.02), (0.1, 0.0, 0.02))
    assert ga.force_detach() == 'detach'
    assert ga.force_detach() is None


def test_invalid_thresholds():
    with pytest.raises(ValueError):
        GraspAssist(close_threshold=0.0, open_threshold=0.1, attach_distance=0.01)
