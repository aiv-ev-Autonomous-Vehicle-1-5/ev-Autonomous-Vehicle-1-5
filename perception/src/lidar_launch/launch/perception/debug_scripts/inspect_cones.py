#!/usr/bin/env python3
"""make_cylinder 출력 검증 스크립트.

/perception/cones (ev_msgs/ConeArray) 토픽을 구독하여
각 프레임의 콘 정보(위치, 반지름, 높이, 신뢰도, 라벨)를 출력한다.
첫 프레임 수신 후 자동 종료 (single-shot).
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ev_msgs.msg import ConeArray


class InspectConesNode(Node):
    def __init__(self, topic: str = '/perception/cones'):
        super().__init__('inspect_cones')
        self._topic = topic
        self._done = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            ConeArray, self._topic, self._callback, qos)

        self.get_logger().info(f'inspect_cones waiting on {self._topic} ...')

    def _callback(self, msg: ConeArray):
        if self._done:
            return

        stamp = msg.header.stamp
        n = len(msg.cones)

        self.get_logger().info(
            f'=== ConeArray stamp={stamp.sec}.{stamp.nanosec:09d}  '
            f'frame_id={msg.header.frame_id}  cones={n} ===')

        if n == 0:
            self.get_logger().warn('no cones in this frame')
            return

        for i, c in enumerate(msg.cones):
            self.get_logger().info(
                f'  [{i:3d}] label={c.label:4d}  '
                f'pos=({c.position.x:+7.3f}, {c.position.y:+7.3f}, {c.position.z:+7.3f})  '
                f'r={c.radius:.3f}  h={c.height:.3f}  '
                f'conf={c.confidence:.3f}')

        # 통계
        avg_conf = sum(c.confidence for c in msg.cones) / n
        avg_r = sum(c.radius for c in msg.cones) / n
        avg_h = sum(c.height for c in msg.cones) / n
        self.get_logger().info(
            f'  --- summary: {n} cones  '
            f'avg_r={avg_r:.3f}  avg_h={avg_h:.3f}  avg_conf={avg_conf:.3f}')

        self._done = True
        self.get_logger().info('single-shot inspect finished. exiting...')
        rclpy.shutdown()


def main():
    topic = '/perception/cones'
    if len(sys.argv) >= 2:
        topic = sys.argv[1]

    rclpy.init()
    node = InspectConesNode(topic)

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
