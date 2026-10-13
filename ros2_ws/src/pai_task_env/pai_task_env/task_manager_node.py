"""物体（と置き場所）の位置のランダム化・位置の publish・成功判定を行うタスク管理ノード."""
from __future__ import annotations

import json
import os
import random
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

from crane_plus_kinematics import ARM_JOINT_NAMES, load_kinematics
from pai_task_env.frames import BaseFrame
from pai_task_env.grasp_assist import GraspAssist
from pai_task_env.sampler import ObjectSampler, PlaceSampler, SamplingError, sample_scene
from pai_task_env.success_checker import SuccessTracker


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        p = self.declare_parameter
        self.mode = p('mode', 'pick').value
        seed = p('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)
        self.world = p('gazebo.world_name', 'default').value
        self.object_model = p('gazebo.object_model', 'target_cube').value
        self.place_model = p('gazebo.place_model', 'place_zone').value
        odom_topic = p('object_odom_topic', '/task/object_odom').value
        self.timeout_ms = p('gazebo.service_timeout_ms', 2000).value
        self.frame = BaseFrame(origin=tuple(p('base_origin', [0.0, 0.0, 1.015]).value),
                               yaw=p('base_yaw', 0.0).value)
        self.object_size = p('object.size', 0.025).value
        self.spawn_margin = p('object.spawn_z_margin', 0.005).value
        self.sampler = ObjectSampler(
            x=tuple(p('sampler.x', [0.10, 0.20]).value),
            y=tuple(p('sampler.y', [-0.08, 0.12]).value),
            r=tuple(p('sampler.r', [0.10, 0.22]).value),
            place_xy=tuple(p('place.xy', [0.12, -0.12]).value),
            min_dist_from_place=p('sampler.min_dist_from_place', 0.08).value,
            max_attempts=p('sampler.max_attempts', 1000).value,
        )
        self.place_radius = p('place.radius', 0.03).value
        self.place_sampler = None
        # launch 引数 randomize_place はパラメータファイル（ノード名指定）より優先度が低く place.randomize を
        # 上書きできないため、別名のパラメータで受ける
        if p('place.randomize', False).value or p('randomize_place', False).value:
            self.place_sampler = PlaceSampler(
                x=tuple(p('place.sampler.x', [0.04, 0.16]).value),
                y=tuple(p('place.sampler.y', [-0.14, 0.14]).value),
                r=tuple(p('place.sampler.r', [0.09, 0.165]).value),
                max_attempts=self.sampler.max_attempts,
            )
        self.place_xy = self.sampler.place_xy
        self.rest_z = self.object_size / 2.0
        self.tracker = SuccessTracker(
            place_xy=self.place_xy, place_radius=self.place_radius, rest_z=self.rest_z,
            z_tolerance=p('success.z_tolerance', 0.01).value,
            lift_height=p('success.lift_height', 0.02).value,
            hold_time=p('success.hold_time', 0.5).value,
            mode=self.mode,
            reach_threshold=p('success.reach_threshold', 0.02).value,
        )
        randomize_on_start = p('randomize_on_start', True).value
        status_rate = p('status_rate', 10.0).value
        kin_cfg = p('kinematics_config', '').value or os.path.join(
            get_package_share_directory('crane_plus_kinematics'), 'config', 'kinematics.yaml')
        self.kin = load_kinematics(kin_cfg)
        self.hand_joint = p('hand_joint_name', 'crane_plus_joint_hand').value
        self.grasp_assist = None
        if p('grasp_assist.enabled', True).value:
            self.grasp_assist = GraspAssist(
                close_threshold=p('grasp_assist.close_threshold', 0.12).value,
                open_threshold=p('grasp_assist.open_threshold', 0.0).value,
                attach_distance=p('grasp_assist.attach_distance', 0.02).value,
            )

        self.object_xyz = None
        self.ee_xyz = None
        self.status = {'lifted': False, 'in_place_zone': False, 'success': False}
        self.episode = 0

        self.obj_pub = self.create_publisher(PoseStamped, '/task/object_pose', 10)
        self.place_pub = self.create_publisher(PoseStamped, '/task/place_pose', 10)
        self.status_pub = self.create_publisher(String, '/task/status', 10)
        self.success_pub = self.create_publisher(Bool, '/task/success', 10)
        self.attach_pub = self.create_publisher(Empty, '/task/gz/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/task/gz/detach', 10)
        self.create_subscription(Odometry, odom_topic, self.on_object_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_service(Trigger, '/task/reset', self.on_reset)
        # 評価用: 物体を指定位置（crane_plus_base 基準の x, y）に置いてリセット
        self.create_subscription(Point, '/task/reset_to', self.on_reset_to, 10)
        # 評価用: 物体と置き場所の両方を指定位置に置いてリセット [物体 x, y, 置き場所 x, y]
        self.create_subscription(Float64MultiArray, '/task/reset_to_scene', self.on_reset_to_scene, 10)
        self.create_timer(1.0 / status_rate, self.publish_status)
        if randomize_on_start:
            self._startup_timer = self.create_timer(3.0, self._startup_reset)
        self.get_logger().info(f'task_manager_node started (mode={self.mode}, '
                               f'randomize_place={self.place_sampler is not None})')

    # ---- Gazebo 操作 ----
    def _set_pose(self, model: str, world_xyz) -> bool:
        req = (f'name: "{model}", position: {{x: {world_xyz[0]:.5f}, y: {world_xyz[1]:.5f}, '
               f'z: {world_xyz[2]:.5f}}}, orientation: {{w: 1.0}}')
        cmd = ['ign', 'service', '-s', f'/world/{self.world}/set_pose',
               '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', str(self.timeout_ms), '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=self.timeout_ms / 1000.0 + 2.0)
        except subprocess.TimeoutExpired:
            return False
        ok = 'data: true' in out.stdout
        if not ok:
            self.get_logger().warn(f'set_pose {model} failed: {out.stdout} {out.stderr}')
        return ok

    def _apply_grasp_action(self, action):
        if action == 'attach':
            self.attach_pub.publish(Empty())
            self.get_logger().info('grasp assist: attach')
        elif action == 'detach':
            self.detach_pub.publish(Empty())
            self.get_logger().info('grasp assist: detach')

    def reset_task(self, xy=None, place_xy=None):
        """物体（と置き場所）を置き直す. 指定がなければサンプルする（置き場所は固定設定なら固定位置）."""
        if self.grasp_assist is not None:
            self.grasp_assist.force_detach()
            self.detach_pub.publish(Empty())
        if xy is None:
            (x, y), place = sample_scene(self.sampler, self.place_sampler, self.rng)
            place_xy = place_xy or place
        else:
            x, y = xy
        self.place_xy = tuple(place_xy) if place_xy is not None else self.sampler.place_xy
        obj_ok = self._set_pose(self.object_model, self.frame.base_to_world(
            (x, y, self.rest_z + self.spawn_margin)))
        px, py = self.place_xy
        place_ok = self._set_pose(self.place_model, self.frame.base_to_world((px, py, 0.0005)))
        self.tracker.set_place(self.place_xy)
        self.tracker.reset()
        self.episode += 1
        return obj_ok and place_ok, (x, y)

    def _startup_reset(self):
        self._startup_timer.cancel()
        ok, xy = self.reset_task()
        self.get_logger().info(f'initial object placement ok={ok} xy={xy}')

    # ---- callbacks ----
    def on_reset(self, _req, res):
        try:
            ok, (x, y) = self.reset_task()
        except SamplingError as e:
            res.success, res.message = False, str(e)
            return res
        res.success = ok
        res.message = f'object=({x:.3f}, {y:.3f}) place=({self.place_xy[0]:.3f}, {self.place_xy[1]:.3f})'
        return res

    def on_reset_to(self, msg: Point):
        ok, (x, y) = self.reset_task((msg.x, msg.y))
        self.get_logger().info(f'reset_to ({x:.3f}, {y:.3f}) ok={ok}')

    def on_reset_to_scene(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().error(f'reset_to_scene needs [object x, y, place x, y]: {list(msg.data)}')
            return
        ox, oy, px, py = msg.data
        ok, _ = self.reset_task((ox, oy), (px, py))
        self.get_logger().info(f'reset_to_scene object=({ox:.3f}, {oy:.3f}) place=({px:.3f}, {py:.3f}) ok={ok}')

    def on_object_odom(self, msg: Odometry):
        """OdometryPublisher の world 姿勢を crane_plus_base 基準に変換して publish."""
        t = msg.pose.pose.position
        self.object_xyz = self.frame.world_to_base((t.x, t.y, t.z))
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'crane_plus_base'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.object_xyz
        ps.pose.orientation = msg.pose.pose.orientation
        self.obj_pub.publish(ps)

    def on_joint_state(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if not all(n in pos for n in ARM_JOINT_NAMES):
            return
        ee = self.kin.forward([pos[n] for n in ARM_JOINT_NAMES])
        self.ee_xyz = (ee.x, ee.y, ee.z)
        if self.grasp_assist is not None and self.object_xyz is not None and self.hand_joint in pos:
            self._apply_grasp_action(
                self.grasp_assist.update(pos[self.hand_joint], self.ee_xyz, self.object_xyz))

    def publish_status(self):
        now = self.get_clock().now()
        place = PoseStamped()
        place.header.stamp = now.to_msg()
        place.header.frame_id = 'crane_plus_base'
        place.pose.position.x, place.pose.position.y = self.place_xy
        place.pose.orientation.w = 1.0
        self.place_pub.publish(place)
        if self.object_xyz is not None:
            self.status = self.tracker.update(now.nanoseconds * 1e-9, self.object_xyz, self.ee_xyz)
        self.success_pub.publish(Bool(data=self.status['success']))
        self.status_pub.publish(String(data=json.dumps({
            **self.status,
            'mode': self.mode,
            'episode': self.episode,
            'object': [round(v, 4) for v in self.object_xyz] if self.object_xyz else None,
            'place': [round(v, 4) for v in self.place_xy],
            'grasped': bool(self.grasp_assist and self.grasp_assist.attached),
        })))


def main():
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
