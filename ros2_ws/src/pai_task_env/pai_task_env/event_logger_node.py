"""方策の ON/OFF とタスク成功の時刻（壁時計, UNIX 秒）を CSV に記録する（画面録画した動画の切り出し用）.

ros2 run pai_task_env event_logger --ros-args -p output:=~/dataset/media/clips/run.events_abs.csv
"""
from __future__ import annotations

import csv
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pai_task_env.frame_recorder import EventTracker


class EventLogger(Node):
    def __init__(self):
        super().__init__('event_logger')
        self.output = Path(os.path.expanduser(self.declare_parameter('output', '~/dataset/media/events.csv').value))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.tracker = EventTracker()
        self.events = []
        self.create_subscription(String, '/policy/status', lambda m: self.on_status('policy', m, 'enabled'), 10)
        self.create_subscription(String, '/task/status', lambda m: self.on_status('success', m, 'success'), 10)

    def on_status(self, key: str, msg: String, field: str):
        ev = self.tracker.update(key, bool(json.loads(msg.data).get(field)))
        if ev is not None:
            self.events.append((round(time.time(), 3), ev))

    def close(self):
        with open(self.output, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time', 'event'])
            w.writerows(self.events)


def main():
    rclpy.init()
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
