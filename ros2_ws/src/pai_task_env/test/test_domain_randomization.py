import math
import random

import pytest

from pai_task_env.domain_randomization import (
    SceneSampler,
    bbox_center_to_position,
    cylinder_bbox,
    hsv_to_rgb,
    polyline_clear_of,
)
from pai_task_env.vision import CameraModel

SIZE = (320, 240)
CAM = CameraModel.from_hfov(320, 240, 1.0, position=[0.12, 0.0, 0.45], rpy=[0.0, 1.5708, 0.0])


def test_cylinder_bbox_contains_projected_center_and_is_symmetric_below_camera():
    # カメラ直下の円柱は画像中心付近に、ほぼ左右対称の枠になる
    u0, v0, u1, v1 = cylinder_bbox(CAM, (0.12, 0.0, 0.02), radius=0.02, height=0.04, image_size=SIZE)
    cu, cv = CAM.project((0.12, 0.0, 0.04))
    assert u0 < cu < u1 and v0 < cv < v1
    assert (u0 + u1) / 2 == pytest.approx(160, abs=1.0)
    # 直径 4cm、上面までの距離 0.41m → 約 0.04 * fx / 0.41 px
    assert u1 - u0 == pytest.approx(0.04 * CAM.fx / 0.41, rel=0.1)


def test_cylinder_bbox_widens_toward_image_edge_due_to_side_wall():
    u0, v0, u1, v1 = cylinder_bbox(CAM, (0.12, 0.10, 0.02), 0.02, 0.04, SIZE)
    w_center = cylinder_bbox(CAM, (0.12, 0.0, 0.02), 0.02, 0.04, SIZE)
    assert (u1 - u0) > (w_center[2] - w_center[0])


def test_cylinder_bbox_is_clipped_to_image():
    u0, v0, u1, v1 = cylinder_bbox(CAM, (0.12, 0.30, 0.02), 0.02, 0.04, SIZE)
    assert u0 >= 0 and u1 <= 320 and v0 >= 0 and v1 <= 240


def test_hsv_to_rgb_primary_colors():
    assert hsv_to_rgb(0.0, 1.0, 1.0) == pytest.approx((1.0, 0.0, 0.0))
    assert hsv_to_rgb(1 / 3, 1.0, 1.0) == pytest.approx((0.0, 1.0, 0.0))


def test_polyline_clear_of():
    line = [(0.0, 0.0), (0.2, 0.0)]
    assert not polyline_clear_of(line, (0.1, 0.01), 0.03)
    assert polyline_clear_of(line, (0.1, 0.05), 0.03)
    assert not polyline_clear_of(line, (0.21, 0.0), 0.03)   # 端点付近


CFG = {
    'object': {'saturation': [0.5, 1.0], 'value': [0.4, 1.0]},
    'distractors': {'count': [0, 3], 'size': [0.02, 0.05], 'min_dist': 0.06,
                    'x': [0.0, 0.25], 'y': [-0.15, 0.15]},
    'table_tint': [0.6, 1.0],
    'light': {'intensity': [0.5, 1.2], 'max_tilt': 0.6},
}


def test_scene_sampler_reproducible_and_respects_ranges():
    obj = (0.12, 0.03)
    a = SceneSampler(CFG).sample(random.Random(3), obj)
    b = SceneSampler(CFG).sample(random.Random(3), obj)
    assert a == b
    for _ in range(50):
        s = SceneSampler(CFG).sample(random.Random(_), obj)
        assert 0 <= len(s.distractors) <= 3
        for d in s.distractors:
            assert math.dist(d.xy, obj) >= 0.06
            assert 0.02 <= d.size <= 0.05
        assert 0.5 <= s.light_intensity <= 1.2
        assert all(0.0 <= c <= 1.0 for c in s.object_rgb)
        # 光は下向き
        assert s.light_direction[2] < 0


@pytest.mark.parametrize('xy', [(0.12, 0.0), (0.09, 0.11), (0.16, -0.05), (0.10, 0.07)])
def test_bbox_center_to_position_inverts_cylinder_projection(xy):
    # 囲み枠の中心から、側面の見え方も含めて円柱の中心位置を逆算できる（0.5mm 以内）
    u0, v0, u1, v1 = cylinder_bbox(CAM, (xy[0], xy[1], 0.02), 0.02, 0.04, SIZE)
    x, y = bbox_center_to_position(CAM, ((u0 + u1) / 2, (v0 + v1) / 2), 0.02, 0.04, SIZE, center_z=0.02)
    assert math.dist((x, y), xy) < 0.0005


def test_naive_backprojection_of_bbox_center_is_biased_off_axis():
    # 比較: 枠の中心を物体中心高さの平面へそのまま投影すると、画像の端ほどずれる（補正が必要な理由）
    xy = (0.09, 0.11)
    u0, v0, u1, v1 = cylinder_bbox(CAM, (xy[0], xy[1], 0.02), 0.02, 0.04, SIZE)
    naive = CAM.pixel_to_plane((u0 + u1) / 2, (v0 + v1) / 2, 0.02)
    assert math.dist(naive[:2], xy) > 0.001


# ---- 置き場所（置き場所もランダムに変え、検出器で見る。実験12） ----
CFG_PLACE = {**CFG, 'place': {'saturation': [0.4, 1.0], 'value': [0.45, 1.0]},
             'distractors': {**CFG['distractors'], 'min_dist_place': 0.09}}


def test_scene_sampler_keeps_distractors_off_the_place():
    obj, place = (0.12, 0.03), (0.08, -0.1)
    for seed in range(50):
        s = SceneSampler(CFG_PLACE).sample(random.Random(seed), obj, place_xy=place)
        for d in s.distractors:
            assert math.dist(d.xy, place) >= 0.09
            assert math.dist(d.xy, obj) >= 0.06


def test_scene_sampler_randomizes_place_color_only_when_configured():
    s = SceneSampler(CFG_PLACE).sample(random.Random(0), (0.12, 0.03), place_xy=(0.08, -0.1))
    assert s.place_rgb is not None and all(0.0 <= c <= 1.0 for c in s.place_rgb)
    assert SceneSampler(CFG).sample(random.Random(0), (0.12, 0.03)).place_rgb is None


def test_disc_bbox_for_place_is_centered_below_camera():
    # 置き場所（厚さ 1mm の円板）の枠: カメラ直下なら中心の投影と枠の中心がほぼ一致する
    u0, v0, u1, v1 = cylinder_bbox(CAM, (0.12, 0.0, 0.0005), 0.035, 0.001, SIZE)
    uc, vc = CAM.project((0.12, 0.0, 0.0005))
    assert abs((u0 + u1) / 2 - uc) < 0.5 and abs((v0 + v1) / 2 - vc) < 0.5
