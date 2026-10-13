"""方策評価結果の集計（ROS 非依存）."""
from __future__ import annotations

import csv
import math
import random
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class TrialResult:
    trial: int
    success: bool
    lifted: bool
    duration: float      # 成功までの時間、失敗時はタイムアウトまでの時間 [s]
    object_x: float
    object_y: float
    # 開始姿勢のホームからの手先オフセット [m]（開始姿勢への頑健性評価用。通常は 0）
    start_dx: float = 0.0
    start_dy: float = 0.0
    start_dz: float = 0.0
    # 置き場所（置き場所をランダム化した評価のみ。固定なら空）
    place_x: Optional[float] = None
    place_y: Optional[float] = None


def parse_positions(flat: Sequence[float], width: int = 2) -> List[Tuple[float, ...]]:
    """ROS パラメータの平坦なリストを width 個ずつの組にする.

    width=2: [x0, y0, x1, y1, ...] → 物体位置 (x, y)
    width=4: [ox0, oy0, px0, py0, ...] → (物体 x, y, 置き場所 x, y)（置き場所もランダム化する評価）
    """
    if len(flat) % width:
        raise ValueError(f'positions must have a multiple of {width} values: {len(flat)}')
    return [tuple(float(v) for v in flat[i:i + width]) for i in range(0, len(flat), width)]


def sample_start_offsets(n: int, max_xyz: Sequence[float], seed: int) -> List[Tuple[float, float, float]]:
    """開始姿勢の手先オフセットを各軸 [-max, +max] の一様分布から再現可能に生成する."""
    rng = random.Random(seed)
    return [tuple(rng.uniform(-m, m) if m > 0 else 0.0 for m in max_xyz) for _ in range(n)]


class SettleDetector:
    """関節が静止したか（直近 window 秒の各関節の変動幅が tolerance 未満か）を判定する.

    方策を開始する時点の姿勢を学習データのホーム姿勢と揃えるために使う
    （ホームへの移動途中で開始すると、正面付近の物体で把持位置がずれる。2026-10-11 の分析）。
    """

    def __init__(self, window: float, tolerance: float):
        self.window = window
        self.tolerance = tolerance
        self.reset()

    def reset(self) -> None:
        self._hist: List[Tuple[float, List[float]]] = []

    def update(self, t: float, q: Sequence[float]) -> None:
        self._hist.append((t, list(q)))
        # 窓の判定に必要な最古の 1 点（窓の開始以前の最新点）は残す
        while len(self._hist) > 1 and self._hist[1][0] <= t - self.window:
            self._hist.pop(0)

    @property
    def settled(self) -> bool:
        if not self._hist or self._hist[-1][0] - self._hist[0][0] < self.window:
            return False
        cols = list(zip(*(q for _, q in self._hist)))
        return all(max(c) - min(c) < self.tolerance for c in cols)


def wilson_interval(successes: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """成功率の Wilson スコア信頼区間（試行数が少なくても破綻しない）."""
    if n == 0:
        return (0.0, 0.0)
    p = successes / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def summarize(results: Sequence[TrialResult]) -> dict:
    n = len(results)
    succ = [r for r in results if r.success]
    lo, hi = wilson_interval(len(succ), n)
    return {
        'trials': n,
        'successes': len(succ),
        'success_rate': len(succ) / n if n else 0.0,
        'ci95_low': lo,
        'ci95_high': hi,
        'lift_rate': sum(r.lifted for r in results) / n if n else 0.0,
        'mean_success_time': sum(r.duration for r in succ) / len(succ) if succ else None,
    }


def write_csv(path: str | Path, results: List[TrialResult]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[fl.name for fl in fields(TrialResult)])
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))
