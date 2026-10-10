import math

import numpy as np
import pytest

from pai_task_env.vision import CameraModel, red_mask, mask_centroid, rpy_to_matrix


def _down_camera():
    # crane_plus_base 基準 (0.12, 0, 0.5) から真下を向くカメラ（Gazebo のカメラは +x が光軸）
    return CameraModel(fx=500.0, fy=500.0, cx=320.0, cy=240.0,
                       position=(0.12, 0.0, 0.5), rpy=(0.0, math.pi / 2, 0.0))


def test_rpy_to_matrix_pitch_down():
    r = rpy_to_matrix(0.0, math.pi / 2, 0.0)
    # カメラ +x（光軸）が base の -z を向く
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 0, -1], abs=1e-9)


def test_principal_point_hits_point_below_camera():
    cam = _down_camera()
    assert cam.pixel_to_plane(320.0, 240.0, plane_z=0.02) == pytest.approx((0.12, 0.0, 0.02), abs=1e-9)


def test_project_and_back_project_roundtrip():
    cam = _down_camera()
    for p in [(0.15, 0.05, 0.02), (0.08, -0.1, 0.02), (0.2, 0.12, 0.02)]:
        u, v = cam.project(p)
        assert cam.pixel_to_plane(u, v, plane_z=0.02) == pytest.approx(p, abs=1e-9)


def test_image_axes_orientation():
    cam = _down_camera()
    # base の +y（左）は画像の左（u 減少）、+x（前）は画像の上（v 減少）
    u0, v0 = cam.project((0.12, 0.0, 0.02))
    u_left, _ = cam.project((0.12, 0.05, 0.02))
    _, v_fwd = cam.project((0.17, 0.0, 0.02))
    assert u_left < u0
    assert v_fwd < v0


def test_from_hfov():
    cam = CameraModel.from_hfov(width=640, height=480, hfov=1.0, position=(0, 0, 1), rpy=(0, math.pi / 2, 0))
    assert cam.fx == pytest.approx(320.0 / math.tan(0.5))
    assert cam.cx == pytest.approx(320.0) and cam.cy == pytest.approx(240.0)


def test_red_mask_and_centroid():
    img = np.zeros((48, 64, 3), dtype=np.uint8)
    img[:, :] = (120, 80, 50)            # 机の茶色
    img[10:20, 30:40] = (230, 25, 25)    # 赤い物体
    m = red_mask(img, min_red=150, max_other_ratio=0.5)
    assert m.sum() == 100
    c = mask_centroid(m, min_pixels=20)
    assert c == pytest.approx((34.5, 14.5))


def test_centroid_none_when_too_small():
    m = np.zeros((10, 10), dtype=bool)
    m[0, 0] = True
    assert mask_centroid(m, min_pixels=5) is None
