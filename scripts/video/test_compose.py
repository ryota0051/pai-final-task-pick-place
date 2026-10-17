"""compose.py のモデル表（場面ごとに使うモデルを強調する上部の帯・一覧の画面）のテスト.

コンテナ内で: cd ~/final_task && python3 -m pytest -q scripts/video
"""
import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compose import ACCENT, STRIP_H, Composer, validate  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((ROOT / 'docs' / 'report' / 'video.yaml').read_text(encoding='utf-8'))
needs_font = pytest.mark.skipif(not os.path.exists(CFG['font']), reason='日本語フォントはコンテナにのみある')


def test_video_yaml_models_are_consistent():
    validate(CFG)
    ids = [m['id'] for m in CFG['models']]
    assert len(ids) == len(set(ids))
    assert all({'id', 'name', 'input', 'data'} <= set(m) for m in CFG['models'])
    # 比較の場面には使うモデルが書いてある
    assert any(item.get('models') for item in CFG['timeline'])


def test_validate_rejects_unknown_model():
    cfg = {'models': [{'id': 'M0', 'name': 'a', 'input': 'b', 'data': 'c'}],
           'timeline': [{'type': 'card', 'models': ['M9']}]}
    with pytest.raises(ValueError):
        validate(cfg)


@needs_font
def test_model_strip_highlights_only_active_columns(tmp_path):
    comp = Composer(CFG, tmp_path)
    ids = [m['id'] for m in CFG['models']]
    from PIL import Image
    img = Image.open(comp.model_strip_layer([ids[2]])).convert('RGB')
    assert img.size == (comp.W, comp.H)
    centers = comp.strip_column_centers()
    assert len(centers) == len(ids)
    y = STRIP_H - 10   # 列の下寄り（文字のない所）の色を見る
    assert img.getpixel((int(centers[2]), y)) == ACCENT
    assert img.getpixel((int(centers[0]), y)) != ACCENT
    # 帯より下（映像の部分）は透明のまま
    rgba = Image.open(comp.model_strip_layer([ids[2]]))
    assert rgba.getpixel((comp.W // 2, comp.H // 2))[3] == 0


@needs_font
def test_models_card_is_full_frame(tmp_path):
    from PIL import Image
    comp = Composer(CFG, tmp_path)
    assert Image.open(comp.models_card()).size == (comp.W, comp.H)
