"""撮影用カメラ（/task/presentation/image）の画像を静止画・動画に保存する（動画・レポート素材用）.

静止画: ros2 run pai_task_env frame_recorder --ros-args -p output:=~/dataset/media/scene.png -p snapshot:=true
動画  : ros2 run pai_task_env frame_recorder --ros-args -p output:=~/dataset/media/run.mp4 -p duration:=20.0
        （duration <= 0 なら Ctrl-C まで）。方策の ON/OFF・タスク成功の時刻（動画内の秒）を <output>.events.csv に書く
"""
from __future__ import annotations

import csv
import json
import os
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pai_task_env.frame_recorder import EventTracker, FrameRateLimiter


class FrameRecorder(Node):
    def __init__(self):
        super().__init__('frame_recorder')
        p = self.declare_parameter
        self.output = Path(os.path.expanduser(p('output', '~/dataset/media/out.mp4').value))
        self.snapshot = p('snapshot', False).value
        self.fps = p('fps', 20.0).value
        self.duration = p('duration', 0.0).value
        self.limiter = FrameRateLimiter(self.fps)
        self.bridge = CvBridge()
        self.writer = None
        self.frames = 0
        self.t0 = None
        self.done = False
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, p('topic', '/task/presentation/image').value, self.on_image, 5)
        self.tracker = EventTracker()
        self.events = []
        self.create_subscription(String, '/policy/status', lambda m: self.on_status('policy', m, 'enabled'), 10)
        self.create_subscription(String, '/task/status', lambda m: self.on_status('success', m, 'success'), 10)

    def on_image(self, msg: Image):
        if self.done:
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.snapshot:
            cv2.imwrite(str(self.output), img)
            self.get_logger().info(f'saved {self.output}')
            self.done = True
            return
        if not self.limiter.accept(now - self.t0):
            return
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(str(self.output), cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (w, h))
        self.writer.write(img)
        self.frames += 1
        if self.duration > 0 and now - self.t0 >= self.duration:
            self.done = True

    def on_status(self, key: str, msg: String, field: str):
        if self.writer is None or self.done:
            return
        ev = self.tracker.update(key, bool(json.loads(msg.data).get(field)))
        if ev is not None:
            self.events.append((round(self.frames / self.fps, 2), ev))

    def close(self):
        if self.writer is not None:
            self.writer.release()
            with open(f'{self.output}.events.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['time', 'event'])
                w.writerows(self.events)
            self.get_logger().info(f'saved {self.output} ({self.frames} frames, {len(self.events)} events)')


def main():
    rclpy.init()
    node = FrameRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
