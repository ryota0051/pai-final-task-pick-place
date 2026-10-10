"""ブラウザ等から受け取った手先目標を IK で関節指令に変換して publish するノード."""
from __future__ import annotations

import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from crane_plus_kinematics import ARM_JOINT_NAMES, Workspace, load_kinematics
from pai_web_teleop.ros_utils import make_trajectory, pitch_to_quaternion, quaternion_to_pitch
from pai_web_teleop.teleop_state import EETeleopState, gripper_to_angle


class EETeleopNode(Node):
    def __init__(self):
        super().__init__('ee_teleop_node')
        p = self.declare_parameter
        kin_cfg = p('kinematics_config', '').value or os.path.join(
            get_package_share_directory('crane_plus_kinematics'), 'config', 'kinematics.yaml')
        self.rate = p('publish_rate', 20.0).value
        self.traj_time = p('trajectory_time', 0.1).value
        self.frame_id = p('frame_id', 'crane_plus_base').value
        self.gripper_joint = p('gripper_joint_name', 'crane_plus_joint_hand').value
        self.gripper_open = p('gripper_open', -0.5236).value
        self.gripper_close = p('gripper_close', 0.1745).value
        self.max_lin = p('max_linear_speed', 0.08).value
        self.max_pitch = p('max_pitch_speed', 1.0).value
        self.vel_timeout = p('velocity_timeout', 0.3).value
        home_ee = list(p('home_ee', [0.10, 0.0, 0.08, 1.134]).value)
        pitch_search_step = p('pitch_search_step', 0.05).value
        ws = Workspace(
            x=tuple(p('workspace.x', [0.0, 0.26]).value),
            y=tuple(p('workspace.y', [-0.2, 0.2]).value),
            z=tuple(p('workspace.z', [-0.01, 0.25]).value),
            r=tuple(p('workspace.r', [0.06, 0.26]).value),
        )
        pitch_range = tuple(p('pitch_range', [-0.5, 1.5708]).value)
        self.enabled = p('start_enabled', True).value
        arm_topic = p('arm_topic', '/crane_plus_arm_controller/joint_trajectory').value
        gripper_topic = p('gripper_topic', '/crane_plus_gripper_controller/joint_trajectory').value

        kin = load_kinematics(kin_cfg)
        # ホーム姿勢もテレオペと同じ pitch 緩和付きで解く（設定値が厳密に解けなくても起動できる）
        self.state = EETeleopState(kin, ws, [0.0] * len(ARM_JOINT_NAMES), pitch_range,
                                   pitch_search_step)
        if not self.state.set_target(*home_ee):
            raise RuntimeError(f'home_ee {home_ee} has no IK solution')
        self.state.home_joints = list(self.state.joints)
        self.state.reset_home()
        self.gripper = 0.0
        self.velocity = (0.0, 0.0, 0.0, 0.0)
        self.last_vel_time = None
        self.latest_joint_state = None

        self.arm_pub = self.create_publisher(JointTrajectory, arm_topic, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, gripper_topic, 10)
        self.ee_pub = self.create_publisher(PoseStamped, '/teleop/ee_state', 10)
        self.status_pub = self.create_publisher(String, '/teleop/status', 10)
        self.create_subscription(PoseStamped, '/teleop/ee_target', self.on_target, 10)
        self.create_subscription(Twist, '/teleop/ee_velocity', self.on_velocity, 10)
        self.create_subscription(Float64, '/teleop/gripper', self.on_gripper, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_service(Trigger, '/teleop/home', self.on_home)
        self.create_service(SetBool, '/teleop/enable', self.on_enable)
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info(f'ee_teleop_node started (enabled={self.enabled})')

    # ---- callbacks ----
    def on_target(self, msg: PoseStamped):
        o = msg.pose.orientation
        pos = msg.pose.position
        self.state.set_target(pos.x, pos.y, pos.z, quaternion_to_pitch(o.x, o.y, o.z, o.w))

    def on_velocity(self, msg: Twist):
        def lim(v, m):
            return max(-m, min(m, v))
        self.velocity = (lim(msg.linear.x, self.max_lin), lim(msg.linear.y, self.max_lin),
                         lim(msg.linear.z, self.max_lin), lim(msg.angular.y, self.max_pitch))
        self.last_vel_time = self.get_clock().now()

    def on_gripper(self, msg: Float64):
        self.gripper = min(max(msg.data, 0.0), 1.0)

    def on_joint_state(self, msg: JointState):
        self.latest_joint_state = dict(zip(msg.name, msg.position))

    def on_home(self, _req, res):
        self.state.reset_home()
        self.gripper = 0.0
        res.success = True
        res.message = 'home'
        return res

    def on_enable(self, req, res):
        if req.data and not self.enabled and self.latest_joint_state:
            # 自律実行などの後に再開する場合、現在姿勢から目標を作り直してジャンプを防ぐ
            q = [self.latest_joint_state.get(n, 0.0) for n in ARM_JOINT_NAMES]
            fk = self.state.kin.forward(q)
            if not self.state.set_target(fk.x, fk.y, fk.z, fk.pitch):
                self.state.reset_home()
        self.enabled = bool(req.data)
        res.success = True
        res.message = f'enabled={self.enabled}'
        return res

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_vel_time is not None:
            if (now - self.last_vel_time).nanoseconds * 1e-9 > self.vel_timeout:
                self.velocity = (0.0, 0.0, 0.0, 0.0)
                self.last_vel_time = None
            else:
                self.state.apply_velocity(*self.velocity, dt=1.0 / self.rate)

        if self.enabled:
            self.arm_pub.publish(make_trajectory(ARM_JOINT_NAMES, self.state.joints, self.traj_time))
            angle = gripper_to_angle(self.gripper, self.gripper_open, self.gripper_close)
            self.gripper_pub.publish(make_trajectory([self.gripper_joint], [angle], self.traj_time))

        t = self.state.target
        ee = PoseStamped()
        ee.header.stamp = now.to_msg()
        ee.header.frame_id = self.frame_id
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = t.x, t.y, t.z
        qx, qy, qz, qw = pitch_to_quaternion(t.pitch)
        ee.pose.orientation.x, ee.pose.orientation.y = qx, qy
        ee.pose.orientation.z, ee.pose.orientation.w = qz, qw
        self.ee_pub.publish(ee)
        self.status_pub.publish(String(data=json.dumps({
            'enabled': self.enabled,
            'ik_ok': self.state.last_ik_ok,
            'pitch_relaxed': self.state.pitch_relaxed,
            'requested_pitch': round(self.state.requested_pitch, 4),
            'joints': [round(v, 4) for v in self.state.joints],
            'target': [round(t.x, 4), round(t.y, 4), round(t.z, 4), round(t.pitch, 4)],
            'gripper': self.gripper,
        })))


def main():
    rclpy.init()
    node = EETeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
