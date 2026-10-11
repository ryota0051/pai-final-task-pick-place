"""操作時間の計測ノード（テレオペ方式の比較用）.

ros2 run pai_task_env episode_timer --ros-args --params-file config/episode_timer.yaml -p label:=browser
エピソードごとに「リセット〜動き始め」「動き始め〜成功」を ~/dataset/teleop_time/<label>.csv に追記する。
"""
from __future__ import annotations

import csv
import json
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from pai_task_env.episode_timer import EpisodeTimer


class EpisodeTimerNode(Node):
    def __init__(self):
        super().__init__('episode_timer')
        p = self.declare_parameter
        self.label = p('label', 'teleop').value
        out_dir = Path(os.path.expanduser(p('output_dir', '~/dataset/teleop_time').value))
        self.joints = list(p('joints', ['crane_plus_joint1', 'crane_plus_joint2',
                                        'crane_plus_joint3', 'crane_plus_joint4']).value)
        self.timer = EpisodeTimer(p('motion_threshold', 0.02).value)
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = out_dir / f'{self.label}.csv'
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/task/status', self.on_status, 10)
        self.get_logger().info(f'recording operation time to {self.path}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.joints):
            self.timer.on_joints(self.now(), [pos[n] for n in self.joints])

    def on_status(self, msg: String):
        s = json.loads(msg.data)
        rec = self.timer.on_status(self.now(), s.get('episode'), bool(s.get('success')))
        if rec is None:
            return
        new = not self.path.exists()
        with open(self.path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['label', 'episode', 'idle', 'duration'])
            if new:
                w.writeheader()
            w.writerow({'label': self.label, 'episode': rec['episode'],
                        'idle': round(rec['idle'], 2), 'duration': round(rec['duration'], 2)})
        self.get_logger().info(f"[{self.label}] episode {rec['episode']}: {rec['duration']:.1f} s")


def main():
    rclpy.init()
    node = EpisodeTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
