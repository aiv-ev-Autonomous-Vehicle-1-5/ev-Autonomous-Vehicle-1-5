#!/usr/bin/env python3

from collections import Counter
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class ClusterCounterNode(Node):
    def __init__(self):
        super().__init__('cluster_counter')
        self._latest_total: Optional[int] = None
        self._latest_unique: Optional[int] = None
        self._latest_sample = []
        self._msg_count = 0

        self._sub = self.create_subscription(
            PointCloud2,
            '/pointcloud/clustered',
            self._cb,
            10,
        )
        self._timer = self.create_timer(1.0, self._print_status)
        self.get_logger().info('cluster_counter started (print every 1s)')

    def _cb(self, msg: PointCloud2):
        counts = Counter()
        total = 0
        for point in pc2.read_points(msg, field_names=['cluster_id'], skip_nans=False):
            label = point[0]
            counts[label] += 1
            total += 1

        self._latest_total = total
        self._latest_unique = len(counts)
        self._latest_sample = list(counts.items())[:10]
        self._msg_count += 1

    def _print_status(self):
        if self._latest_total is None:
            self.get_logger().info('clusters: waiting for /pointcloud/clustered ...')
            return

        self.get_logger().info(
            f'clusters(1s) msgs={self._msg_count} '
            f'total={self._latest_total} unique={self._latest_unique} '
            f'sample={self._latest_sample}'
        )


def main():
    rclpy.init()
    node = ClusterCounterNode()
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
