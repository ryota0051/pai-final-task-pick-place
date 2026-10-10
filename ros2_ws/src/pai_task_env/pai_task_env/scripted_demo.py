"""スクリプト化デモで把持・配置エピソードを自動収録する.

1 エピソード: /teleop/home → /task/reset → /recorder/start → 計画実行（/teleop/ee_target, /teleop/gripper）
→ /recorder/stop → /task/status が成功でなければ /recorder/discard_last。

Usage:
    ros2 run pai_task_env scripted_demo --ros-args --params-file <scripted_demo.yaml> -p episodes:=30
"""
from __future__ import annotations

import json
import math
import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from pai_task_env.scripted_policy import PickPlacePlan, PlanParams


class ScriptedDemo(Node):
    def __init__(self):
        super().__init__('scripted_demo')
        p = self.declare_parameter
        self.episodes = p('episodes', 1).value
        self.record = p('record', True).value
        self.rate = p('rate', 20.0).value
        self.settle_time = p('settle_time', 1.5).value
        self.post_time = p('post_time', 1.0).value
        seed = p('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)
        self.base_params = PlanParams(
            z_approach=p('plan.z_approach', 0.08).value,
            z_grasp=p('plan.z_grasp', 0.02).value,
            z_carry=p('plan.z_carry', 0.08).value,
            z_place=p('plan.z_place', 0.025).value,
            pitch=p('plan.pitch', math.pi / 2).value,
            move_speed=p('plan.move_speed', 0.05).value,
            min_segment_time=p('plan.min_segment_time', 0.5).value,
            grasp_hold=p('plan.grasp_hold', 0.8).value,
            release_hold=p('plan.release_hold', 0.6).value,
        )
        self.speed_scale = tuple(p('randomize.speed_scale', [0.8, 1.2]).value)
        self.height_jitter = p('randomize.height_jitter', 0.01).value
        self.xy_noise = p('randomize.xy_noise', 0.003).value
        self.verbose = p('verbose', False).value

        self.target_pub = self.create_publisher(PoseStamped, '/teleop/ee_target', 10)
        self.gripper_pub = self.create_publisher(Float64, '/teleop/gripper', 10)
        self.latest = {}
        for topic, key, typ in [('/task/object_pose', 'obj', PoseStamped),
                                ('/task/place_pose', 'place', PoseStamped),
                                ('/teleop/ee_state', 'ee', PoseStamped)]:
            self.create_subscription(typ, topic, lambda m, k=key: self.latest.__setitem__(k, m), 10)
        self.create_subscription(String, '/task/status',
                                 lambda m: self.latest.__setitem__('status', json.loads(m.data)), 10)
        self.trigger_clients = {name: self.create_client(Trigger, name) for name in
                        ['/teleop/home', '/task/reset', '/recorder/start', '/recorder/stop',
                         '/recorder/discard_last']}

    # ---- helpers ----
    def call(self, name: str) -> bool:
        cli = self.trigger_clients[name]
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'service {name} unavailable')
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn(f'{name}: {res.message if res else "timeout"}')
            return False
        return True

    def spin_for(self, sec: float):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish(self, w):
        m = PoseStamped()
        m.header.frame_id = 'crane_plus_base'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = w.x, w.y, w.z
        m.pose.orientation.y, m.pose.orientation.w = math.sin(w.pitch / 2), math.cos(w.pitch / 2)
        self.target_pub.publish(m)
        self.gripper_pub.publish(Float64(data=float(w.gripper)))

    def run_episode(self, idx: int) -> bool:
        self.call('/teleop/home')
        self.spin_for(self.settle_time)
        if not self.call('/task/reset'):
            return False
        self.latest.pop('obj', None)
        self.spin_for(self.settle_time)
        if 'obj' not in self.latest or 'ee' not in self.latest or 'place' not in self.latest:
            self.get_logger().error('object / ee / place pose not received')
            return False
        o = self.latest['obj'].pose.position
        pl = self.latest['place'].pose.position
        ee = self.latest['ee'].pose.position
        noise = self.xy_noise
        params = self.base_params.randomized(self.rng, self.speed_scale, self.height_jitter)
        plan = PickPlacePlan.create(
            start=(ee.x, ee.y, ee.z),
            obj_xy=(o.x + self.rng.uniform(-noise, noise), o.y + self.rng.uniform(-noise, noise)),
            place_xy=(pl.x + self.rng.uniform(-noise, noise), pl.y + self.rng.uniform(-noise, noise)),
            params=params)

        if self.record and not self.call('/recorder/start'):
            return False
        t0 = time.monotonic()
        period = 1.0 / self.rate
        phase = None
        while True:
            t = time.monotonic() - t0
            w = plan.sample(t)
            if self.verbose and w.phase != phase:
                st = self.latest.get('status', {})
                obj = self.latest['obj'].pose.position
                self.get_logger().info(
                    f'  -> {w.phase}: lifted={st.get("lifted")} grasped={st.get("grasped")} '
                    f'obj=({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})')
                phase = w.phase
            self.publish(w)
            if t > plan.total_duration + self.post_time:
                break
            self.spin_for(period)
        success = bool(self.latest.get('status', {}).get('success'))
        if self.record:
            self.call('/recorder/stop')
            if not success:
                self.call('/recorder/discard_last')
        self.get_logger().info(
            f'episode {idx}: success={success} duration={plan.total_duration:.1f}s '
            f'object=({o.x:.3f}, {o.y:.3f})')
        return success


def main():
    rclpy.init()
    node = ScriptedDemo()
    ok = 0
    try:
        for i in range(node.episodes):
            ok += int(node.run_episode(i))
        node.get_logger().info(f'done: {ok}/{node.episodes} succeeded')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
