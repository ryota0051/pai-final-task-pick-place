import random

from pai_task_env.scene_randomizer import SceneRandomizer

CFG = {
    'object': {'saturation': [0.4, 1.0], 'value': [0.45, 1.0]},
    'distractors': {'count': [3, 3], 'pool_sizes': [0.02, 0.03, 0.04, 0.05], 'min_dist': 0.07,
                    'x': [0.0, 0.24], 'y': [-0.16, 0.16]},
    'table_tint': [0.5, 1.0],
    'light': {'intensity': [0.5, 1.1], 'max_tilt': 0.7},
}


def test_sample_assigns_distinct_pool_boxes_and_their_sizes():
    r = SceneRandomizer(CFG)
    scene, ids = r.sample(random.Random(0), (0.12, 0.0))
    assert len(ids) == len(scene.distractors) == 3
    assert len(set(ids)) == 3
    for k, d in zip(ids, scene.distractors):
        assert d.size == CFG['distractors']['pool_sizes'][k]


def test_sample_is_reproducible():
    a = SceneRandomizer(CFG).sample(random.Random(5), (0.12, 0.0))
    b = SceneRandomizer(CFG).sample(random.Random(5), (0.12, 0.0))
    assert a == b


def test_light_call_keeps_original_range_and_attenuation():
    s, t, req = SceneRandomizer(CFG).light_call(0.8, (-0.5, 0.1, -0.9))
    assert s.endswith('/light_config')
    assert 'range: 1000' in req and 'attenuation_constant: 0.9' in req and 'intensity: 1' in req
