"""方策評価結果の集計（ROS 非依存）."""
from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class TrialResult:
    trial: int
    success: bool
    lifted: bool
    duration: float      # 成功までの時間、失敗時はタイムアウトまでの時間 [s]
    object_x: float
    object_y: float


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
