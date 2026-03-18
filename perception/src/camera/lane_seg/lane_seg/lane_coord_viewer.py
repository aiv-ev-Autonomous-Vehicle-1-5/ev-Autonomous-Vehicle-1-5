#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lane_seg_msgs.msg import LaneCoords

class LaneCoordViewer(Node):
    def __init__(self):
        super().__init__('lane_coord_viewer')
        self.subscription = self.create_subscription(
            LaneCoords, '/lane_coordinates', self.callback, 10)
        self.get_logger().info("lane_coordinates 수신 대기 중...")

    def callback(self, msg):
        n = len(msg.line_x)
        if n == 0:
            self.get_logger().info("차선 미검출")
            return

        self.get_logger().info(f"--- 차선 좌표 ({n}개 포인트) ---")
        for i in range(0, min(n, 10)):  # 최대 10개만 출력
            self.get_logger().info(f"  x={msg.line_x[i]:.3f}m, y={msg.line_y[i]:.3f}m")
        if n > 10:
            self.get_logger().info(f"  ... 외 {n - 10}개")

def main(args=None):
    rclpy.init(args=args)
    node = LaneCoordViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
