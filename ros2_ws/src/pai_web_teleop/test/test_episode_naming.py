from pai_web_teleop.episode_naming import next_episode_index, episode_dir


def test_empty_dir_starts_at_zero(tmp_path):
    assert next_episode_index(tmp_path) == 0


def test_missing_dir_starts_at_zero(tmp_path):
    assert next_episode_index(tmp_path / 'not_exist') == 0


def test_next_after_max(tmp_path):
    for name in ['ep0', 'ep1', 'ep5', 'other', 'epX']:
        (tmp_path / name).mkdir()
    (tmp_path / 'ep9').write_text('file, not dir')
    assert next_episode_index(tmp_path) == 6


def test_custom_prefix(tmp_path):
    (tmp_path / 'trial_3').mkdir()
    assert next_episode_index(tmp_path, prefix='trial_') == 4


def test_episode_dir(tmp_path):
    assert episode_dir(tmp_path, 7) == tmp_path / 'ep7'
