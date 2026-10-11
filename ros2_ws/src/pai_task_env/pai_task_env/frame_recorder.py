"""撮影用カメラの画像を動画に書き出すときのフレーム間引き（ROS 非依存）."""
from __future__ import annotations


class FrameRateLimiter:
    """入力フレームを目標 fps に間引く.

    次に採用する時刻を 1/fps ずつ進めるので、入力の揺らぎでずれが蓄積しない（周期の 1/4 までの早着は許容）。
    大きく遅れた場合は現在時刻から数え直す。
    """

    def __init__(self, fps: float):
        self.period = 1.0 / fps
        self.next_t = None

    def accept(self, t: float) -> bool:
        if self.next_t is not None and t < self.next_t - self.period / 4:
            return False
        if self.next_t is None or t > self.next_t + self.period:
            self.next_t = t + self.period
        else:
            self.next_t += self.period
        return True
