"""1 エピソードの操作時間の計測（ROS 非依存）.

操作方法（ブラウザ手先 UI / 関節キーボードなど）によらず比較できるよう、
指令ではなく関節の動きで開始を判定する: リセット後に関節が motion_threshold 以上動いた時刻から成功までを計る。
"""
from __future__ import annotations

from typing import Optional, Sequence


class EpisodeTimer:
    def __init__(self, motion_threshold: float):
        self.motion_threshold = motion_threshold
        self.episode = None
        self._reset(None, 0.0)

    def _reset(self, episode, t: float) -> None:
        self.episode = episode
        self.t_reset = t
        self.q0: Optional[list] = None
        self.t_start: Optional[float] = None
        self.done = False

    def on_joints(self, t: float, q: Sequence[float]) -> None:
        if self.q0 is None:
            self.q0 = list(q)
        elif self.t_start is None and max(abs(a - b) for a, b in zip(q, self.q0)) >= self.motion_threshold:
            self.t_start = t

    def on_status(self, t: float, episode, success: bool) -> Optional[dict]:
        """成功した瞬間に {'episode', 'idle'（リセット〜動き始め）, 'duration'（動き始め〜成功）} を返す."""
        if episode != self.episode:
            self._reset(episode, t)
        if not success or self.done or self.t_start is None:
            return None
        self.done = True
        return {'episode': episode, 'idle': self.t_start - self.t_reset, 'duration': t - self.t_start}
