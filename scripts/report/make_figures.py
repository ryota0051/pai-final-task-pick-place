"""レポート用のグラフを docs/report/results.yaml から生成する.

uv run --group report python scripts/report/make_figures.py
出力: docs/report/img/*.png
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'pai_task_env'))
from pai_task_env.evaluation import wilson_interval  # noqa: E402

REPORT = ROOT / 'docs' / 'report'
COLORS = ['#9aa5b1', '#2a6fdb']


def _rate_and_err(successes, trials):
    rates, lo, hi = [], [], []
    for s, n in zip(successes, trials):
        r = s / n
        a, b = wilson_interval(s, n)
        rates.append(r * 100)
        lo.append((r - a) * 100)
        hi.append((b - r) * 100)
    return np.array(rates), np.array([lo, hi])


def _finish(ax, title, path):
    ax.set_ylim(0, 115)
    ax.set_ylabel('成功率 [%]')
    ax.set_title(title, fontsize=11)
    ax.grid(axis='y', alpha=0.3)
    ax.spines[['top', 'right']].set_visible(False)
    ax.figure.tight_layout()
    ax.figure.savefig(path, dpi=200)
    plt.close(ax.figure)
    print(f'saved {path}')


def bars(cfg, path, color):
    labels = [b['label'] for b in cfg['bars']]
    rates, err = _rate_and_err([b['successes'] for b in cfg['bars']], [b['trials'] for b in cfg['bars']])
    fig, ax = plt.subplots(figsize=(1.3 * len(labels) + 1.5, 3.2))
    x = np.arange(len(labels))
    ax.bar(x, rates, color=color, yerr=err, capsize=3, ecolor='#555')
    for xi, r, h in zip(x, rates, err[1]):
        ax.text(xi, r + h + 2, f'{r:.0f}%', ha='center', fontsize=9)
    ax.set_xticks(x, labels, fontsize=8.5)
    _finish(ax, cfg['title'], path)


def grouped(cfg, path):
    conds = cfg['conditions']
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    x = np.arange(len(conds))
    w = 0.38
    for k, s in enumerate(cfg['series']):
        rates, err = _rate_and_err(s['successes'], [s['trials']] * len(conds))
        xs = x + (k - 0.5) * w
        ax.bar(xs, rates, w, label=s['label'], color=COLORS[k], yerr=err, capsize=3, ecolor='#555')
        for xi, r, h in zip(xs, rates, err[1]):
            ax.text(xi, r + h + 2, f'{r:.0f}', ha='center', fontsize=8)
    ax.set_xticks(x, conds, fontsize=8.5)
    ax.legend(fontsize=8, loc=cfg.get('legend_loc', 'lower left'))
    _finish(ax, cfg['title'], path)


def main():
    cfg = yaml.safe_load((REPORT / 'results.yaml').read_text(encoding='utf-8'))
    plt.rcParams['font.family'] = cfg['font_family']
    out = REPORT / 'img'
    out.mkdir(parents=True, exist_ok=True)
    bars(cfg['improvements'], out / 'fig_improvements.png', COLORS[1])
    grouped(cfg['start_robustness'], out / 'fig_start_robustness.png')
    bars(cfg['object_corruption'], out / 'fig_object_corruption.png', COLORS[1])
    grouped(cfg['detector_policy'], out / 'fig_detector_policy.png')


if __name__ == '__main__':
    main()
