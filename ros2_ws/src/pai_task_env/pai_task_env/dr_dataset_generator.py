"""物体検出器の学習データを Gazebo で自動生成する（ドメインランダム化, ADR 0008）.

ros2 run pai_task_env dr_dataset_generator --ros-args --params-file config/domain_randomization.yaml -p images:=20
1 枚: 物体をランダム配置（/task/reset）→ 物体色・机の色味・照明・ダミーの箱をランダム化 → 腕を物体を隠さない姿勢へ
→ 俯瞰カメラ画像を保存し、正解の囲み枠（円柱を投影）を COCO 形式で書く。
出力: <output_dir>/images/*.png, annotations.json, meta.jsonl（物体位置・シーン設定）
速度のため、ダミーの箱は大きさ違いを最初に作って使い回し（位置と色だけ変える。実行中に大きさは変えられない）、
シーン変更のサービス呼び出しは並列に行う。腕の姿勢は IK で関節角を求め、物体を隠さないものだけ動かす。
終了時に物体・机・照明を元に戻し、ダミーの箱を消す。
place.enabled のとき（実験12）は置き場所も 2 クラス目（place）として枠を付け、色もランダムにし、腕で隠さない。
タスク環境を randomize_place:=true で起動しておけば、置き場所の位置も毎回変わる。
"""
from __future__ import annotations

import json
import math
import os
import random
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crane_plus_kinematics.kinematics import ARM_JOINT_NAMES, EndEffectorPose, load_kinematics
from pai_task_env.domain_randomization import cylinder_bbox, polyline_clear_of
from pai_task_env.scene_randomizer import SceneRandomizer
from pai_task_env.vision import CameraModel

class DrDatasetGenerator(Node):
    def __init__(self):
        super().__init__('dr_dataset_generator')
        p = self.declare_parameter
        self.n = p('images', 100).value
        self.rng = random.Random(p('seed', 0).value)
        self.out = Path(os.path.expanduser(p('output_dir', '~/dataset/detector/dr').value))
        cam_path = p('camera_config', '').value or os.path.join(
            get_package_share_directory('pai_task_env'), 'config', 'camera.yaml')
        c = yaml.safe_load(open(cam_path))['object_detector_node']['ros__parameters']['camera']
        self.size = (c['width'], c['height'])
        self.cam = CameraModel.from_hfov(c['width'], c['height'], c['hfov'], c['position'], c['rpy'])
        self.world = p('world', 'default').value
        self.object_model = p('object_model', 'target_cube').value
        self.table_model = p('table_model', 'Table').value
        self.table_visual = p('table_visual', 'visual1').value
        self.light = p('light_name', 'sun').value
        self.obj_size = p('object_size', 0.04).value
        # 置き場所（2 クラス目）。厚さ 1mm の円板として枠を投影する
        self.place = {k: p(f'place.{k}', d).value for k, d in
                      [('enabled', False), ('model', 'place_zone'), ('radius', 0.035), ('height', 0.001)]}
        self.settle = p('settle_time', 1.0).value
        self.arm = {k: p(f'arm.{k}', d).value for k, d in
                    [('x', [0.03, 0.2]), ('y', [-0.16, 0.16]), ('z', [0.06, 0.18]), ('pitch', 1.134),
                     ('clearance', 0.05), ('home_ratio', 0.3), ('speed', 0.08)]}
        rnd = 'randomize.'
        place_cfg = ({'saturation': p(rnd + 'place.saturation', [0.4, 1.0]).value,
                      'value': p(rnd + 'place.value', [0.45, 1.0]).value} if self.place['enabled'] else None)
        self.randomizer = SceneRandomizer({
            'place': place_cfg,
            'object': {'saturation': p(rnd + 'object.saturation', [0.4, 1.0]).value,
                       'value': p(rnd + 'object.value', [0.45, 1.0]).value},
            'distractors': {'count': p(rnd + 'distractors.count', [0, 3]).value,
                            'pool_sizes': p(rnd + 'distractors.pool_sizes', [0.02, 0.03, 0.04, 0.05]).value,
                            'min_dist': p(rnd + 'distractors.min_dist', 0.07).value,
                            'min_dist_place': p(rnd + 'distractors.min_dist_place', 0.09).value,
                            'x': p(rnd + 'distractors.x', [0.0, 0.24]).value,
                            'y': p(rnd + 'distractors.y', [-0.16, 0.16]).value},
            'table_tint': p(rnd + 'table_tint', [0.5, 1.0]).value,
            'light': {'intensity': p(rnd + 'light.intensity', [0.5, 1.1]).value,
                      'max_tilt': p(rnd + 'light.max_tilt', 0.7).value},
        }, world=self.world, object_model=self.object_model, table_model=self.table_model,
            table_visual=self.table_visual, light_name=self.light, place_model=self.place['model'])
        self.kin = load_kinematics(os.path.join(
            get_package_share_directory('crane_plus_kinematics'), 'config', 'kinematics.yaml'))
        self.home = list(p('arm.home_ee', [0.10, 0.0, 0.08]).value)
        self.bridge = CvBridge()
        self.latest = {}
        self.create_subscription(Image, '/task/camera/image', lambda m: self.latest.__setitem__('image', m), 2)
        self.create_subscription(PoseStamped, '/task/object_pose', lambda m: self.latest.__setitem__('obj', m), 10)
        self.create_subscription(PoseStamped, '/task/place_pose', lambda m: self.latest.__setitem__('place', m), 10)
        self.create_subscription(JointState, '/joint_states', lambda m: self.latest.__setitem__('joints', m), 10)
        self.create_subscription(String, '/task/status',
                                 lambda m: self.latest.__setitem__('status', json.loads(m.data)), 10)
        self.target_pub = self.create_publisher(PoseStamped, '/teleop/ee_target', 10)
        self.reset_cli = self.create_client(Trigger, '/task/reset')
        self.ee = tuple(self.home)
        self.randomizer.setup()

    # ---- ROS ----
    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, cli):
        cli.wait_for_service(timeout_sec=10.0)
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

    def move_ee(self, target):
        """テレオペの速度制限に合わせた時間だけ手先目標を送る."""
        m = PoseStamped()
        m.header.frame_id = 'crane_plus_base'
        pitch = self.arm['pitch']
        m.pose.position.x, m.pose.position.y, m.pose.position.z = target
        m.pose.orientation.y, m.pose.orientation.w = math.sin(pitch / 2), math.cos(pitch / 2)
        end = time.monotonic() + math.dist(self.ee, target) / self.arm['speed'] + 0.3
        while time.monotonic() < end:
            self.target_pub.publish(m)
            self.spin_for(0.05)
        self.ee = tuple(target)

    def arm_clear(self, q, obj_xy, place_xy=None) -> bool:
        """腕（根元・肘・手首・手先を結ぶ線）が物体（と置き場所）から clearance 以上離れているか.

        置き場所は物体より半径が大きいので、半径の差だけ余分に離す。
        """
        ee = self.kin.forward(q)
        pts = [(0.0, 0.0), self.kin.elbow_position(q)[:2], self.kin.wrist_position(q)[:2], (ee.x, ee.y)]
        if not polyline_clear_of(pts, obj_xy, self.arm['clearance']):
            return False
        extra = self.place['radius'] - self.obj_size / 2
        return place_xy is None or polyline_clear_of(pts, place_xy, self.arm['clearance'] + extra)

    def clear_pose(self, target, obj_xy, place_xy=None) -> bool:
        """IK で関節角を求めて事前に判定する（実際に動かす前の絞り込み）."""
        q = self.kin.inverse(EndEffectorPose(*target, pitch=self.arm['pitch']))
        return q is not None and self.arm_clear(q, obj_xy, place_xy)

    def actual_clear(self, obj_xy, place_xy=None) -> bool:
        """実際の関節角で判定する（テレオペが pitch を緩めた場合などに備えて撮影直前に確認）."""
        pos = dict(zip(self.latest['joints'].name, self.latest['joints'].position))
        return self.arm_clear([pos[n] for n in ARM_JOINT_NAMES], obj_xy, place_xy)

    def place_arm(self, obj_xy, place_xy=None) -> bool:
        if self.rng.random() < self.arm['home_ratio'] and self.clear_pose(tuple(self.home), obj_xy, place_xy):
            self.move_ee(tuple(self.home))
            return True
        for _ in range(30):
            target = (self.rng.uniform(*self.arm['x']), self.rng.uniform(*self.arm['y']),
                      self.rng.uniform(*self.arm['z']))
            if self.clear_pose(target, obj_xy, place_xy):
                self.move_ee(target)
                return True
        return False

    def fresh_image(self, after):
        while True:
            self.spin_for(0.05)
            img = self.latest.get('image')
            if img is not None and img.header.stamp.sec + img.header.stamp.nanosec * 1e-9 > after:
                return img

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- メイン ----
    def run(self):
        img_dir = self.out / 'images'
        img_dir.mkdir(parents=True, exist_ok=True)
        categories = [{'id': 0, 'name': 'target'}]
        if self.place['enabled']:
            categories.append({'id': 1, 'name': 'place'})
        coco = {'images': [], 'annotations': [], 'categories': categories}
        meta = open(self.out / 'meta.jsonl', 'w')
        i, t0 = 0, time.monotonic()
        while i < self.n:
            before = self.latest.get('status', {}).get('episode')
            self.call(self.reset_cli)
            while self.latest.get('status', {}).get('episode') == before:
                self.spin_for(0.05)
            self.spin_for(0.3)
            o = self.latest['obj'].pose.position
            place_xy = None
            if self.place['enabled']:
                pl = self.latest['place'].pose.position
                place_xy = (pl.x, pl.y)
            if not self.place_arm((o.x, o.y), place_xy):
                continue
            scene, box_ids = self.randomizer.sample(self.rng, (o.x, o.y), place_xy)
            self.randomizer.apply(scene, box_ids)
            self.spin_for(self.settle)
            if not self.actual_clear((o.x, o.y), place_xy):
                continue
            img = self.fresh_image(self.now())
            boxes = [(0, cylinder_bbox(self.cam, (o.x, o.y, o.z), self.obj_size / 2, self.obj_size, self.size))]
            if place_xy is not None:
                ph = self.place['height']
                boxes.append((1, cylinder_bbox(self.cam, (*place_xy, ph / 2), self.place['radius'], ph, self.size)))
            if any(b[2] - b[0] < 4 or b[3] - b[1] < 4 for _, b in boxes):
                continue
            name = f'{i:05d}.png'
            cv2.imwrite(str(img_dir / name), self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8'))
            coco['images'].append({'id': i, 'file_name': name, 'width': self.size[0], 'height': self.size[1]})
            for cat, (x0, y0, x1, y1) in boxes:
                coco['annotations'].append({'id': len(coco['annotations']), 'image_id': i, 'category_id': cat,
                                            'iscrowd': 0, 'bbox': [x0, y0, x1 - x0, y1 - y0],
                                            'area': (x1 - x0) * (y1 - y0)})
            meta.write(json.dumps({'image_id': i, 'object': [o.x, o.y, o.z], 'place': place_xy,
                                   'scene': asdict(scene)}) + '\n')
            meta.flush()
            i += 1
            if i % 50 == 0:
                rate = (time.monotonic() - t0) / i
                self.get_logger().info(f'{i}/{self.n} images ({rate:.1f} s/image)')
                (self.out / 'annotations.json').write_text(json.dumps(coco))
        (self.out / 'annotations.json').write_text(json.dumps(coco))
        meta.close()
        self.get_logger().info(f'done: {i} images -> {self.out}')

    def restore(self):
        self.randomizer.restore()
        self.move_ee(tuple(self.home))


def main():
    rclpy.init()
    node = DrDatasetGenerator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.restore()
        node.destroy_node()
        rclpy.try_shutdown()
