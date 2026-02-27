#!/usr/bin/env python3
"""make_bbox 출력 검증 스크립트.

/perception/bboxes (ev_msgs/BBoxArray) 토픽을 구독하여
각 프레임의 bbox 정보(위치, 바운딩박스 크기, 신뢰도, 라벨)를 출력한다.
첫 프레임 수신 후 자동 종료 (single-shot).
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ev_msgs.msg import BBoxArray


class InspectBBoxesNode(Node):
    def __init__(self, topic: str = '/perception/bboxes'):
        super().__init__('inspect_bboxes')
        self._topic = topic
        self._done = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            BBoxArray, self._topic, self._callback, qos)

        self.get_logger().info(f'inspect_bboxes waiting on {self._topic} ...')

    def _callback(self, msg: BBoxArray):
        if self._done:
            return

        stamp = msg.header.stamp
        n = len(msg.bboxes)

        self.get_logger().info(
            f'=== BBoxArray stamp={stamp.sec}.{stamp.nanosec:09d}  '
            f'frame_id={msg.header.frame_id}  bboxes={n} ===')

        if n == 0:
            self.get_logger().warn('no bboxes in this frame')
            return

        for i, b in enumerate(msg.bboxes):
            self.get_logger().info(
                f'  [{i:3d}] label={b.label:4d}  '
                f'pos=({b.position.x:+7.3f}, {b.position.y:+7.3f}, {b.position.z:+7.3f})  '
                f'bbox=({b.size_x:.3f}, {b.size_y:.3f}, {b.size_z:.3f})  '
                f'conf={b.confidence:.3f}')

        # 통계
        avg_conf = sum(b.confidence for b in msg.bboxes) / n
        avg_sx = sum(b.size_x for b in msg.bboxes) / n
        avg_sy = sum(b.size_y for b in msg.bboxes) / n
        avg_sz = sum(b.size_z for b in msg.bboxes) / n
        self.get_logger().info(
            f'  --- summary: {n} bboxes  '
            f'avg_bbox=({avg_sx:.3f}, {avg_sy:.3f}, {avg_sz:.3f})  avg_conf={avg_conf:.3f}')

        self._done = True
        self.get_logger().info('single-shot inspect finished. exiting...')
        rclpy.shutdown()


def main():
    topic = '/perception/bboxes'
    if len(sys.argv) >= 2:
        topic = sys.argv[1]

    rclpy.init()
    node = InspectBBoxesNode(topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
