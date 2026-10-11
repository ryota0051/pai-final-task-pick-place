import pytest

from pai_task_env.episode_timer import EpisodeTimer


def test_measures_from_first_motion_to_success():
    t = EpisodeTimer(motion_threshold=0.01)
    t.on_status(0.0, episode=1, success=False)
    t.on_joints(0.0, [0.0, 0.0])
    t.on_joints(5.0, [0.005, 0.0])        # まだ動いていない
    t.on_joints(6.0, [0.02, 0.0])         # 動き始め
    assert t.on_status(10.0, episode=1, success=False) is None
    rec = t.on_status(16.5, episode=1, success=True)
    assert rec == {'episode': 1, 'idle': pytest.approx(6.0), 'duration': pytest.approx(10.5)}
    # 成功は 1 エピソード 1 回だけ記録
    assert t.on_status(17.0, episode=1, success=True) is None


def test_new_episode_resets_timer():
    t = EpisodeTimer(motion_threshold=0.01)
    t.on_status(0.0, episode=1, success=False)
    t.on_joints(0.0, [0.0])
    t.on_joints(1.0, [0.1])
    t.on_status(2.0, episode=2, success=False)   # リセット（成功せず次へ）
    t.on_joints(2.0, [0.1])
    t.on_joints(3.0, [0.3])
    rec = t.on_status(7.0, episode=2, success=True)
    assert rec['episode'] == 2 and rec['duration'] == pytest.approx(4.0)


def test_success_without_motion_is_ignored():
    t = EpisodeTimer(motion_threshold=0.01)
    t.on_status(0.0, episode=1, success=False)
    t.on_joints(0.0, [0.0])
    assert t.on_status(1.0, episode=1, success=True) is None
