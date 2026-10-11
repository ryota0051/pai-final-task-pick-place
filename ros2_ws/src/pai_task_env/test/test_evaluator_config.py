"""policy_evaluator.yaml と他パッケージの設定の整合性."""
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
TELEOP = PKG.parent / 'pai_web_teleop' / 'config' / 'teleop.yaml'


def _params(path, node):
    return yaml.safe_load(path.read_text(encoding='utf-8'))[node]['ros__parameters']


@pytest.mark.skipif(not TELEOP.exists(), reason='pai_web_teleop が同じワークスペースにない')
def test_start_home_matches_teleop_home():
    ev = _params(PKG / 'config' / 'policy_evaluator.yaml', 'policy_evaluator')
    tel = _params(TELEOP, 'ee_teleop_node')
    assert ev['start']['home_ee'] == tel['home_ee']


def test_start_perturbation_disabled_by_default():
    ev = _params(PKG / 'config' / 'policy_evaluator.yaml', 'policy_evaluator')
    assert ev['start']['offset'] == [0.0, 0.0, 0.0]
    assert ev['start']['moving'] is False
