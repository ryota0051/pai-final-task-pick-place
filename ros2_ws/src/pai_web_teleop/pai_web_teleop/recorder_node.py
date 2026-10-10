"""サービス呼び出しで ros2 bag record を開始・停止するノード（エピソード自動採番）."""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pai_web_teleop.episode_naming import episode_dir, next_episode_index


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        p = self.declare_parameter
        self.root = Path(os.path.expanduser(p('output_root', '~/dataset/rosbags/pick_place').value))
        self.prefix = p('episode_prefix', 'ep').value
        self.topics = list(p('topics', ['/joint_states']).value)
        self.storage = p('storage_id', 'sqlite3').value
        self.stop_timeout = p('stop_timeout', 10.0).value
        self.discard_dirname = p('discard_dirname', '_discarded').value

        self.proc = None
        self.current_dir = None
        self.start_time = None
        self.last_dir = None

        self.status_pub = self.create_publisher(String, '/recorder/status', 10)
        self.create_service(Trigger, '/recorder/start', self.on_start)
        self.create_service(Trigger, '/recorder/stop', self.on_stop)
        self.create_service(Trigger, '/recorder/discard_last', self.on_discard)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f'recorder_node: root={self.root}, topics={self.topics}')

    @property
    def recording(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def on_start(self, _req, res):
        if self.recording:
            res.success, res.message = False, f'already recording {self.current_dir.name}'
            return res
        self.root.mkdir(parents=True, exist_ok=True)
        idx = next_episode_index(self.root, self.prefix)
        self.current_dir = episode_dir(self.root, idx, self.prefix)
        cmd = ['ros2', 'bag', 'record', '-s', self.storage, '-o', str(self.current_dir),
               *self.topics]
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.start_time = time.monotonic()
        self.get_logger().info(f'record start: {self.current_dir}')
        res.success, res.message = True, self.current_dir.name
        return res

    def _stop_proc(self):
        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        try:
            self.proc.wait(timeout=self.stop_timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()

    def on_stop(self, _req, res):
        if not self.recording:
            res.success, res.message = False, 'not recording'
            return res
        self._stop_proc()
        elapsed = time.monotonic() - self.start_time
        self.last_dir = self.current_dir
        self.get_logger().info(f'record stop: {self.current_dir} ({elapsed:.1f}s)')
        res.success, res.message = True, f'{self.current_dir.name} {elapsed:.1f}s'
        self.proc = None
        return res

    def on_discard(self, _req, res):
        """直前のエピソードを削除せず退避ディレクトリへ移動する（採番対象外になる）."""
        if self.recording or self.last_dir is None or not self.last_dir.exists():
            res.success, res.message = False, 'nothing to discard'
            return res
        dst_root = self.root / self.discard_dirname
        dst_root.mkdir(exist_ok=True)
        dst = dst_root / f'{self.last_dir.name}_{int(time.time())}'
        shutil.move(str(self.last_dir), dst)
        res.success, res.message = True, f'moved {self.last_dir.name} -> {dst}'
        self.last_dir = None
        return res

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'recording': self.recording,
            'episode': self.current_dir.name if self.recording else None,
            'elapsed': round(time.monotonic() - self.start_time, 1) if self.recording else 0.0,
            'next_episode': f'{self.prefix}{next_episode_index(self.root, self.prefix)}',
            'last_episode': self.last_dir.name if self.last_dir else None,
        })))

    def destroy_node(self):
        if self.recording:
            self._stop_proc()
        super().destroy_node()


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
