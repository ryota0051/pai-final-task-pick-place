"""rosbag エピソードディレクトリの自動採番（ROS 非依存）."""
from __future__ import annotations

import re
from pathlib import Path


def next_episode_index(root: str | Path, prefix: str = 'ep') -> int:
    """root 直下の `<prefix><N>` ディレクトリの最大 N + 1 を返す。無ければ 0."""
    root = Path(root)
    if not root.is_dir():
        return 0
    pattern = re.compile(rf'^{re.escape(prefix)}(\d+)$')
    indices = [int(m.group(1)) for p in root.iterdir()
               if p.is_dir() and (m := pattern.match(p.name))]
    return max(indices) + 1 if indices else 0


def episode_dir(root: str | Path, index: int, prefix: str = 'ep') -> Path:
    return Path(root) / f'{prefix}{index}'
