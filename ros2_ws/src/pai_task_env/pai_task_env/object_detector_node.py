"""俯瞰カメラ画像から赤い物体を検出し、crane_plus_base 基準の物体位置を推定するノード.

真値（/task/object_pose）の代わりに推論ノードへ与えることで、認識を含めた現実的な評価を行う。
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pai_task_env.vision import CameraModel, mask_centroid, red_mask


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        p = self.declare_parameter
        image_topic = p('image_topic', '/task/camera/image').value
        output_topic = p('output_topic', '/task/object_pose_est').value
        self.cam = CameraModel.from_hfov(
            p('camera.width', 320).value, p('camera.height', 240).value, p('camera.hfov', 1.0).value,
            list(p('camera.position', [0.12, 0.0, 0.45]).value),
            list(p('camera.rpy', [0.0, 1.5708, 0.0]).value))
        self.min_red = p('detection.min_red', 120).value
        self.max_other_ratio = p('detection.max_other_ratio', 0.5).value
        self.min_pixels = p('detection.min_pixels', 15).value
        self.plane_z = p('plane_z', 0.04).value
        self.output_z = p('output_z', 0.02).value
        self.hold_last = p('hold_last', True).value

        self.last = None
        self.detected = False
        self.pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.status_pub = self.create_publisher(String, '/task/detector_status', 10)
        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.get_logger().info(f'object_detector_node: {image_topic} -> {output_topic}')

    def on_image(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'unsupported encoding {msg.encoding}', throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        if msg.encoding == 'bgr8':
            img = img[..., ::-1]
        mask = red_mask(img, self.min_red, self.max_other_ratio)
        c = mask_centroid(mask, self.min_pixels)
        self.detected = c is not None
        if c is not None:
            x, y, _ = self.cam.pixel_to_plane(c[0], c[1], self.plane_z)
            self.last = (x, y, self.output_z)
        if self.last is not None and (self.detected or self.hold_last):
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = 'crane_plus_base'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.last
            ps.pose.orientation.w = 1.0
            self.pub.publish(ps)
        self.status_pub.publish(String(data=json.dumps({
            'detected': self.detected, 'pixels': int(mask.sum()),
            'estimate': None if self.last is None else [round(v, 4) for v in self.last]})))


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
