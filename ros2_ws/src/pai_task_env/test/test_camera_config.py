"""camera.yaml と world SDF のカメラ設定が一致していることを確認する."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
BASE_Z_IN_WORLD = 1.015


def _sdf_camera():
    root = ET.parse(PKG / 'worlds' / 'pick_place.sdf').getroot()
    model = root.find(".//model[@name='overhead_camera']")
    assert model is not None
    pose = [float(v) for v in model.find('pose').text.split()]
    cam = model.find('.//sensor/camera')
    return pose, {
        'hfov': float(cam.find('horizontal_fov').text),
        'width': int(cam.find('image/width').text),
        'height': int(cam.find('image/height').text),
    }


def test_camera_config_matches_world():
    cfg = yaml.safe_load(open(PKG / 'config' / 'camera.yaml', encoding='utf-8'))['object_detector_node']['ros__parameters']
    pose, intr = _sdf_camera()
    cam = cfg['camera']
    assert pose[0:2] == pytest.approx(cam['position'][0:2])
    assert pose[2] == pytest.approx(cam['position'][2] + BASE_Z_IN_WORLD)
    assert pose[3:6] == pytest.approx(cam['rpy'], abs=1e-3)
    assert intr['hfov'] == pytest.approx(cam['hfov'])
    assert (intr['width'], intr['height']) == (cam['width'], cam['height'])


def test_camera_sees_sampling_area():
    from pai_task_env.vision import CameraModel
    cfg = yaml.safe_load(open(PKG / 'config' / 'camera.yaml', encoding='utf-8'))['object_detector_node']['ros__parameters']
    task = yaml.safe_load(open(PKG / 'config' / 'task.yaml', encoding='utf-8'))['task_manager_node']['ros__parameters']
    c = cfg['camera']
    cam = CameraModel.from_hfov(c['width'], c['height'], c['hfov'], c['position'], c['rpy'])
    xs, ys = task['sampler']['x'], task['sampler']['y']
    corners = [(x, y) for x in xs for y in ys] + [tuple(task['place']['xy'])]
    for x, y in corners:
        u, v = cam.project((x, y, cfg['plane_z']))
        assert 0 <= u < c['width'] and 0 <= v < c['height'], (x, y, u, v)
    assert math.isfinite(u)
