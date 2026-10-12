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
import math
import os
import random
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from pai_task_env.evaluation import (
    SettleDetector, TrialResult, parse_positions, sample_start_offsets, summarize, write_csv)
from pai_task_env.sampler import ObjectSampler, PlaceSampler, sample_positions
from pai_task_env.scene_randomizer import SceneRandomizer


class PolicyEvaluator(Node):
    def __init__(self):
        super().__init__('policy_evaluator')
        p = self.declare_parameter
        self.trials = p('trials', 20).value
        self.timeout = p('timeout', 30.0).value
        self.settle_time = p('settle_time', 2.0).value
        # ホームへの移動完了待ち（関節が静止してから開始する）。固定時間だけでは移動途中で開始することがある
        self.settle = SettleDetector(p('settle.window', 0.5).value, p('settle.tolerance', 0.002).value)
        self.settle_timeout = p('settle.timeout', 10.0).value
        self.label = p('label', 'policy').value
        self.output_dir = Path(os.path.expanduser(p('output_dir', '~/dataset/eval').value))
        # カメラ推定を使う場合: 腕をカメラ視野外へ退避 → リセット → 検出を待つ → ホーム（遮蔽対策）
        self.observe = p('observe.enabled', False).value
        self.observe_ee = list(p('observe.ee', [0.02, -0.15, 0.15, 1.134]).value)
        self.observe_move_time = p('observe.move_time', 2.0).value
        self.observe_timeout = p('observe.timeout', 5.0).value
        # 物体位置の固定リスト（条件間で同じ配置を比較するため）。-1 なら毎回ランダム
        seed = p('positions_seed', -1).value
        # 置き場所もランダム化する評価（タスク環境を randomize_place:=true で起動しておく）。
        # 位置リストは (物体 x, y, 置き場所 x, y) になり、/task/reset_to_scene で両方を置き直す
        self.randomize_place = p('randomize_place', False).value
        task_cfg = p('task_config', '').value or os.path.join(
            get_package_share_directory('pai_task_env'), 'config', 'task.yaml')
        self.positions = None
        if seed >= 0:
            with open(task_cfg, encoding='utf-8') as f:
                tp = yaml.safe_load(f)['task_manager_node']['ros__parameters']
            sampler = ObjectSampler.from_dict({**tp['sampler'], 'place_xy': tp['place']['xy']})
            place_sampler = PlaceSampler.from_dict(tp['place']['sampler']) if self.randomize_place else None
            self.positions = sample_positions(sampler, self.trials, seed, place_sampler)
        # 物体位置の直接指定 [x0, y0, x1, y1, ...]（特定の領域の走査用）。指定時は seed より優先し、試行数も上書き
        # 置き場所もランダム化する場合は [物体 x0, y0, 置き場所 x0, y0, ...]
        explicit = parse_positions(p('positions', Parameter.Type.DOUBLE_ARRAY).value or [],
                                   width=4 if self.randomize_place else 2)
        if explicit:
            self.positions = explicit
            self.trials = len(explicit)
        # 開始姿勢の摂動（頑健性評価）: ホーム手先姿勢から offset だけずらした姿勢で開始する。
        # moving=true ならずらした姿勢からホームへ戻る途中（moving_delay 秒後）で開始する
        self.home_ee = list(p('start.home_ee', [0.10, 0.0, 0.08, 1.134]).value)
        offset_max = list(p('start.offset', [0.0, 0.0, 0.0]).value)
        self.start_moving = p('start.moving', False).value
        self.start_moving_delay = p('start.moving_delay', 0.3).value
        self.start_move_time = p('start.move_time', 1.5).value
        self.start_offsets = None
        # シーンのランダム化（物体色・机・照明・ダミーの箱。検出器の頑健性評価, ADR 0008）
        self.randomizer = None
        if p('scene_randomization.enabled', False).value:
            dr_cfg = p('scene_randomization.config', '').value or os.path.join(
                get_package_share_directory('pai_task_env'), 'config', 'domain_randomization.yaml')
            with open(dr_cfg, encoding='utf-8') as f:
                rnd = yaml.safe_load(f)['dr_dataset_generator']['ros__parameters']['randomize']
            # 置き場所の色も変えるのは置き場所を検出する評価のときだけ（従来の評価条件を変えない）
            if not p('scene_randomization.place', False).value:
                rnd = {k: v for k, v in rnd.items() if k != 'place'}
            self.randomizer = SceneRandomizer(rnd)
            self.randomizer.setup()
            self.scene_rng = random.Random(p('scene_randomization.seed', 0).value)
        if any(m > 0 for m in offset_max):
            self.start_offsets = sample_start_offsets(self.trials, offset_max, p('start.offset_seed', 0).value)
        self.status = {}
        self.detector = {}
        self.obj = None
        self.create_subscription(String, '/task/status',
                                 lambda m: self.status.update(json.loads(m.data)), 10)
        self.create_subscription(String, '/task/detector_status',
                                 lambda m: self.detector.update(json.loads(m.data)), 10)
        self.target_pub = self.create_publisher(PoseStamped, '/teleop/ee_target', 10)
        self.reset_to_pub = self.create_publisher(Point, '/task/reset_to', 10)
        self.reset_scene_pub = self.create_publisher(Float64MultiArray, '/task/reset_to_scene', 10)
        self.create_subscription(
            JointState, '/joint_states',
            lambda m: self.settle.update(time.monotonic(), list(m.position)), 10)
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

    def wait_settled(self) -> bool:
        self.settle.reset()
        end = time.monotonic() + self.settle_timeout
        while time.monotonic() < end:
            self.spin_for(0.05)
            if self.settle.settled:
                return True
        self.get_logger().warn('arm did not settle at home before the trial')
        return False

    def move_ee(self, x, y, z, pitch, duration):
        m = PoseStamped()
        m.header.frame_id = 'crane_plus_base'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.y, m.pose.orientation.w = math.sin(pitch / 2), math.cos(pitch / 2)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.target_pub.publish(m)
            self.spin_for(0.05)

    def wait_detection(self) -> bool:
        end = time.monotonic() + self.observe_timeout
        while time.monotonic() < end:
            self.spin_for(0.1)
            d = self.detector
            if d.get('detected_in_episode') and d.get('episode') == self.status.get('episode'):
                return True
        return False

    def reset_object(self, i):
        if self.positions is None:
            self.trigger('/task/reset')
            return
        before = self.status.get('episode')
        pos = self.positions[i]
        if len(pos) == 4:
            self.reset_scene_pub.publish(Float64MultiArray(data=[float(v) for v in pos]))
        else:
            self.reset_to_pub.publish(Point(x=float(pos[0]), y=float(pos[1]), z=0.0))
        end = time.monotonic() + 5.0
        while time.monotonic() < end and self.status.get('episode') == before:
            self.spin_for(0.05)

    def reset_and_randomize(self, i):
        """物体を置き直し、有効ならシーンの見た目を変える.

        配置が事前に分かる場合は見た目を先に変える。置き直した後に変えると、変わるまでの間に
        検出器が元の見た目（赤い物体）を見て推定値を保持してしまい、評価が甘くなる（2026-10-11 に発見）。
        """
        known = self.positions is not None
        if known:
            self.randomize_scene(i)
        self.reset_object(i)
        if not known:
            self.randomize_scene(i)

    def randomize_scene(self, i):
        """有効ならシーンをランダムに変える（物体位置はダミーと重ならないよう考慮）."""
        if self.randomizer is None:
            return
        xy = self.positions[i][:2] if self.positions is not None else (self.obj.x, self.obj.y)
        place = None
        if self.positions is not None and len(self.positions[i]) == 4:
            place = self.positions[i][2:]
        elif self.randomize_place and self.status.get('place'):
            place = self.status['place']
        scene, box_ids = self.randomizer.sample(self.scene_rng, xy, place)
        self.randomizer.apply(scene, box_ids)

    def perturb_start(self, i):
        """開始姿勢をホームからずらす（無効なら何もしない）. 適用したオフセットを返す."""
        if self.start_offsets is None:
            return (0.0, 0.0, 0.0)
        dx, dy, dz = self.start_offsets[i]
        x, y, z, pitch = self.home_ee
        self.move_ee(x + dx, y + dy, z + dz, pitch, duration=self.start_move_time)
        self.wait_settled()
        if self.start_moving:
            self.trigger('/teleop/home')
            self.spin_for(self.start_moving_delay)
        return (dx, dy, dz)

    def run_trial(self, i) -> TrialResult:
        self.set_bool('/policy/enable', False)
        self.set_bool('/teleop/enable', True)
        if self.observe:
            self.move_ee(*self.observe_ee, duration=self.observe_move_time)
            self.reset_and_randomize(i)
            self.spin_for(0.5)
            if not self.wait_detection():
                self.get_logger().warn('object was not detected before the trial')
            self.trigger('/teleop/home')
            self.status = {}
            self.wait_settled()
            self.spin_for(self.settle_time)
        else:
            self.trigger('/teleop/home')
            self.spin_for(self.settle_time)
            self.reset_and_randomize(i)
            self.status = {}
            self.spin_for(self.settle_time)
            self.wait_settled()
        ox, oy = (self.obj.x, self.obj.y) if self.obj else (float('nan'), float('nan'))
        place = self.status.get('place') if self.randomize_place else None
        offset = self.perturb_start(i)

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
                             duration=round(duration, 2), object_x=round(ox, 4), object_y=round(oy, 4),
                             start_dx=round(offset[0], 4), start_dy=round(offset[1], 4),
                             start_dz=round(offset[2], 4),
                             place_x=place[0] if place else None, place_y=place[1] if place else None)
        self.get_logger().info(f'[{self.label}] trial {i}: {result}')
        return result

    def run(self):
        try:
            results = [self.run_trial(i) for i in range(self.trials)]
        finally:
            if self.randomizer is not None:
                self.randomizer.restore()
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
