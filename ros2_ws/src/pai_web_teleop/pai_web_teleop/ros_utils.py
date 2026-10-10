"""ROS メッセージと内部表現の変換ヘルパ."""
from __future__ import annotations

import math

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def pitch_to_quaternion(pitch: float):
    """y 軸回り回転 (水平から下向き正) を (x, y, z, w) で返す."""
    return (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))


def quaternion_to_pitch(qx: float, qy: float, qz: float, qw: float) -> float:
    """y 軸回り回転のみを仮定して pitch を取り出す."""
    return 2.0 * math.atan2(qy, qw)


def make_trajectory(joint_names, positions, time_from_start: float) -> JointTrajectory:
    jt = JointTrajectory()
    jt.joint_names = list(joint_names)
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in positions]
    sec = int(time_from_start)
    pt.time_from_start = Duration(sec=sec, nanosec=int((time_from_start - sec) * 1e9))
    jt.points.append(pt)
    return jt
