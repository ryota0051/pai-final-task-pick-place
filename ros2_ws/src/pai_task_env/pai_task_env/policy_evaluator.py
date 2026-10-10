"""学習済み方策を N 回試行して成功率を測る.

1 試行: 方策停止 → テレオペ有効化・ホーム → /task/reset → テレオペ無効化 → /policy/reset → /policy/enable
→ /task/status が success になるか timeout まで待つ → 方策停止。結果は CSV と JSON サマリに保存。
推論ノード（pai_training の infer.py）は別途起動しておく。

Usage:
    ros2 run pai_task_env policy_evaluator --ros-args --params-file <policy_evaluator.yaml> \
        -p trials:=20 -p label:=objpos
"""
from __future__ import annotations

import json
import os
import random
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from pai_task_env.evaluation import TrialResult, summarize, write_csv


class PolicyEvaluator(Node):
    def __init__(self):
        super().__init__('policy_evaluator')
        p = self.declare_parameter
        self.trials = p('trials', 20).value
        self.timeout = p('timeout', 30.0).value
        self.settle_time = p('settle_time', 2.0).value
        self.label = p('label', 'policy').value
        self.output_dir = Path(os.path.expanduser(p('output_dir', '~/dataset/eval').value))
        self.status = {}
        self.obj = None
        self.create_subscription(String, '/task/status',
                                 lambda m: self.status.update(json.loads(m.data)), 10)
        self.create_subscription(PoseStamped, '/task/object_pose',
                                 lambda m: setattr(self, 'obj', m.pose.position), 10)
        self.triggers = {n: self.create_client(Trigger, n) for n in
                         ['/teleop/home', '/task/reset', '/policy/reset']}
        self.bools = {n: self.create_client(SetBool, n) for n in ['/teleop/enable', '/policy/enable']}

    def _call(self, client, req, name):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'service {name} unavailable')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def trigger(self, name):
        return self._call(self.triggers[name], Trigger.Request(), name)

    def set_bool(self, name, value):
        return self._call(self.bools[name], SetBool.Request(data=value), name)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_trial(self, i) -> TrialResult:
        self.set_bool('/policy/enable', False)
        self.set_bool('/teleop/enable', True)
        self.trigger('/teleop/home')
        self.spin_for(self.settle_time)
        self.trigger('/task/reset')
        self.status = {}
        self.spin_for(self.settle_time)
        ox, oy = (self.obj.x, self.obj.y) if self.obj else (float('nan'), float('nan'))

        self.set_bool('/teleop/enable', False)
        self.trigger('/policy/reset')
        self.set_bool('/policy/enable', True)
        t0 = time.monotonic()
        success = False
        while time.monotonic() - t0 < self.timeout:
            self.spin_for(0.1)
            if self.status.get('success'):
                success = True
                break
        duration = time.monotonic() - t0
        self.set_bool('/policy/enable', False)
        result = TrialResult(trial=i, success=success, lifted=bool(self.status.get('lifted')),
                             duration=round(duration, 2), object_x=round(ox, 4), object_y=round(oy, 4))
        self.get_logger().info(f'[{self.label}] trial {i}: {result}')
        return result

    def run(self):
        results = [self.run_trial(i) for i in range(self.trials)]
        self.set_bool('/teleop/enable', True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        csv_path = self.output_dir / f'{self.label}_{stamp}.csv'
        write_csv(csv_path, results)
        summary = {'label': self.label, **summarize(results)}
        (self.output_dir / f'{self.label}_{stamp}_summary.json').write_text(json.dumps(summary, indent=2))
        self.get_logger().info(f'summary: {summary} -> {csv_path}')


def main():
    rclpy.init()
    node = PolicyEvaluator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
