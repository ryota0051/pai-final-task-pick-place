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


def test_sample_passes_place_to_scene_sampler():
    cfg = {**CFG, 'place': {'saturation': [0.4, 1.0], 'value': [0.45, 1.0]},
           'distractors': {**CFG['distractors'], 'min_dist_place': 0.09}}
    import math
    for seed in range(20):
        scene, _ = SceneRandomizer(cfg).sample(random.Random(seed), (0.12, 0.0), place_xy=(0.06, -0.12))
        assert scene.place_rgb is not None
        assert all(math.dist(d.xy, (0.06, -0.12)) >= 0.09 for d in scene.distractors)


# ---- 元の見た目に戻す（restore）: 置き場所の不透明度を world の SDF と同じにする ----
def _sdf_place_rgba():
    import re
    from pathlib import Path
    sdf = (Path(__file__).resolve().parents[1] / 'worlds' / 'pick_place.sdf').read_text(encoding='utf-8')
    block = sdf[sdf.index('<model name="place_zone">'):]
    return [float(v) for v in re.search(r'<diffuse>([^<]+)</diffuse>', block).group(1).split()]


def _with_ids():
    r = SceneRandomizer(CFG)
    r.ids = {'object': '1', 'table': '2', 'place': '3', 'boxes': ['4', '5', '6', '7']}
    return r


def test_default_place_color_matches_world_sdf():
    from pai_task_env.scene_randomizer import DEFAULT_PLACE_ALPHA, DEFAULT_PLACE_RGB
    rgba = _sdf_place_rgba()
    assert tuple(rgba[:3]) == DEFAULT_PLACE_RGB
    assert rgba[3] == DEFAULT_PLACE_ALPHA < 1.0


def test_restore_keeps_place_translucent():
    calls = _with_ids().restore_calls(remove_boxes=False)
    place = [req for _, _, req in calls if req.startswith('id: 3,')]
    assert len(place) == 1
    assert place[0].count('a: 0.600') == 2      # ambient と diffuse
    assert 'a: 1' not in place[0]


def test_restore_other_visuals_stay_opaque_and_boxes_removed():
    calls = _with_ids().restore_calls(remove_boxes=True)
    obj = [req for _, _, req in calls if req.startswith('id: 1,')][0]
    assert 'a: 1.000' in obj
    assert sum(s.endswith('/remove') for s, _, _ in calls) == 4


def test_apply_place_color_stays_opaque_during_randomization():
    # 見た目ランダム化の最中の置き場所は不透明のまま（これまでの評価・検出器の学習データと同じ見た目）
    r = _with_ids()
    s, t, req = r.visual_call('3', (0.2, 0.3, 0.4))
    assert 'a: 1.000' in req
