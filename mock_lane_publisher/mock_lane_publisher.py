#!/usr/bin/env python3
"""
모의 차선 경계 발행 노드
- 차량 중심 전방 15m 직선 차선 (폭 1.5m)
- 왼쪽 경계: lane_id=0, y=+0.75m
- 오른쪽 경계: lane_id=1, y=-0.75m
- 토픽: /perception/raw_lane_boundaries (LaneBoundaryArray)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ev_msgs.msg import LaneBoundary, LaneBoundaryArray
from geometry_msgs.msg import Point


class MockLanePublisher(Node):
    def __init__(self):
        super().__init__('mock_lane_publisher')

        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(
            LaneBoundaryArray, '/perception/raw_lane_boundaries', qos_be)

        # 10Hz 발행
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Mock lane publisher started — 15m straight, width 1.5m')

    def timer_callback(self):
        msg = LaneBoundaryArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 0.5m 간격, 0~15m 전방
        xs = [i * 0.5 for i in range(31)]

        # 왼쪽 경계 (lane_id=0, y=+0.75)
        left = LaneBoundary()
        left.header = msg.header
        left.lane_id = 0
        left.confidence = 1.0
        left.points = [Point(x=x, y=0.75, z=0.0) for x in xs]

        # 오른쪽 경계 (lane_id=1, y=-0.75)
        right = LaneBoundary()
        right.header = msg.header
        right.lane_id = 1
        right.confidence = 1.0
        right.points = [Point(x=x, y=-0.75, z=0.0) for x in xs]

        msg.boundaries = [left, right]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockLanePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
