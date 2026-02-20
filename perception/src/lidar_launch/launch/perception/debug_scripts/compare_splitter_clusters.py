#!/usr/bin/env python3

from collections import Counter, defaultdict, deque
from dataclasses import dataclass
import sys
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


StampKey = Tuple[int, int]


@dataclass
class FrameStats:
    stamp: StampKey
    total_points: int
    noise_points: int
    labels: List[int]
    cluster_counts: Counter
    cluster_centroids: Dict[int, Tuple[float, float, float]]


class SplitterCompareNode(Node):
    def __init__(
        self,
        input_topic: str = '/pointcloud/clustered',
        output_topic: str = '/pointcloud/clustered_split',
        max_ids_to_print: int = 0,
    ):
        super().__init__('splitter_compare')

        self._input_topic = input_topic
        self._output_topic = output_topic
        self._max_ids_to_print = max_ids_to_print

        self._input_cache: Dict[StampKey, FrameStats] = {}
        self._output_cache: Dict[StampKey, FrameStats] = {}
        self._input_order: Deque[StampKey] = deque()
        self._output_order: Deque[StampKey] = deque()
        self._max_cache_size = 40
        self._pair_count = 0
        self._done = False
        self._sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sub_in = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._input_cb,
            self._sub_qos,
        )
        self._sub_out = self.create_subscription(
            PointCloud2,
            self._output_topic,
            self._output_cb,
            self._sub_qos,
        )

        self.get_logger().info(
            f'splitter_compare started input={self._input_topic} output={self._output_topic}')

    def _stamp_key(self, msg: PointCloud2) -> StampKey:
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _extract_stats(self, msg: PointCloud2) -> Optional[FrameStats]:
        field_names = {field.name for field in msg.fields}
        required = {'x', 'y', 'z', 'cluster_id'}
        if not required.issubset(field_names):
            self.get_logger().error(
                f"received PointCloud2 without required fields: {sorted(required)}")
            return None

        counts = Counter()
        labels: List[int] = []
        sum_x: Dict[int, float] = defaultdict(float)
        sum_y: Dict[int, float] = defaultdict(float)
        sum_z: Dict[int, float] = defaultdict(float)

        try:
            for point in pc2.read_points(msg, field_names=['x', 'y', 'z', 'cluster_id'], skip_nans=False):
                x, y, z, label_raw = point
                label = int(label_raw)
                labels.append(label)
                counts[label] += 1
                sum_x[label] += float(x)
                sum_y[label] += float(y)
                sum_z[label] += float(z)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'failed to parse cluster_id from PointCloud2: {e}')
            return None

        noise_points = counts.get(-1, 0)
        total_points = len(labels)
        cluster_centroids: Dict[int, Tuple[float, float, float]] = {}
        for cluster_id, n_points in counts.items():
            if n_points <= 0:
                continue
            inv_n = 1.0 / float(n_points)
            cluster_centroids[cluster_id] = (
                sum_x[cluster_id] * inv_n,
                sum_y[cluster_id] * inv_n,
                sum_z[cluster_id] * inv_n,
            )

        return FrameStats(
            stamp=self._stamp_key(msg),
            total_points=total_points,
            noise_points=noise_points,
            labels=labels,
            cluster_counts=counts,
            cluster_centroids=cluster_centroids,
        )

    def _cleanup_cache(
        self,
        cache: Dict[StampKey, FrameStats],
        order: Deque[StampKey],
    ) -> None:
        while len(order) > self._max_cache_size:
            old_key = order.popleft()
            cache.pop(old_key, None)

    def _input_cb(self, msg: PointCloud2) -> None:
        if self._done:
            return
        stats = self._extract_stats(msg)
        if stats is None:
            return

        key = stats.stamp
        self._input_cache[key] = stats
        self._input_order.append(key)
        self._cleanup_cache(self._input_cache, self._input_order)
        self._try_report_pair(key)

    def _output_cb(self, msg: PointCloud2) -> None:
        if self._done:
            return
        stats = self._extract_stats(msg)
        if stats is None:
            return

        key = stats.stamp
        self._output_cache[key] = stats
        self._output_order.append(key)
        self._cleanup_cache(self._output_cache, self._output_order)
        self._try_report_pair(key)

    def _fmt_id_list(self, ids: List[int]) -> str:
        if not ids:
            return '[]'

        ids_sorted = sorted(ids)
        if self._max_ids_to_print <= 0:
            return str(ids_sorted)

        if len(ids_sorted) <= self._max_ids_to_print:
            return str(ids_sorted)

        head = ids_sorted[:self._max_ids_to_print]
        return f'{head} ... (+{len(ids_sorted) - self._max_ids_to_print})'

    def _non_noise_ids(self, stats: FrameStats) -> List[int]:
        return [cluster_id for cluster_id in stats.cluster_counts.keys() if cluster_id >= 0]

    def _log_cluster_xyz(self, title: str, stats: FrameStats) -> None:
        ids = sorted(self._non_noise_ids(stats))
        if not ids:
            self.get_logger().info(f'{title} cluster xyz: []')
            return

        max_items = self._max_ids_to_print if self._max_ids_to_print > 0 else len(ids)
        for cluster_id in ids[:max_items]:
            centroid = stats.cluster_centroids.get(cluster_id)
            if centroid is None:
                continue
            self.get_logger().info(
                f'{title} id={cluster_id} xyz=({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}) '
                f'n={stats.cluster_counts[cluster_id]}')

        if len(ids) > max_items:
            self.get_logger().info(
                f'{title} ... truncated {len(ids) - max_items} clusters '
                f'(increase 3rd argument max_ids_to_print)')

    def _try_report_pair(self, key: StampKey) -> None:
        if self._done:
            return
        in_stats = self._input_cache.get(key)
        out_stats = self._output_cache.get(key)
        if in_stats is None or out_stats is None:
            return

        self._pair_count += 1

        in_ids = self._non_noise_ids(in_stats)
        out_ids = self._non_noise_ids(out_stats)
        in_cluster_count = len(in_ids)
        out_cluster_count = len(out_ids)
        delta = out_cluster_count - in_cluster_count

        stamp_text = f'{key[0]}.{key[1]:09d}'
        self.get_logger().info(
            f'pair#{self._pair_count} stamp={stamp_text} '
            f'clusters in:{in_cluster_count} -> out:{out_cluster_count} (delta={delta:+d}) '
            f'points in:{in_stats.total_points} out:{out_stats.total_points} '
            f'noise in:{in_stats.noise_points} out:{out_stats.noise_points}')
        self.get_logger().info(f'input cluster_ids : {self._fmt_id_list(in_ids)}')
        self.get_logger().info(f'output cluster_ids: {self._fmt_id_list(out_ids)}')
        self._log_cluster_xyz('input', in_stats)
        self._log_cluster_xyz('output', out_stats)

        if in_stats.total_points == out_stats.total_points:
            mapping: Dict[int, Counter] = defaultdict(Counter)

            for in_label, out_label in zip(in_stats.labels, out_stats.labels):
                if in_label < 0 or out_label < 0:
                    continue
                mapping[in_label][out_label] += 1

            split_count = 0
            map_lines = []
            for in_id in sorted(mapping.keys()):
                out_counter = mapping[in_id]
                out_items = sorted(out_counter.items(), key=lambda item: (-item[1], item[0]))
                if len(out_items) > 1:
                    split_count += 1
                parts = [f'{out_id}:{pts}' for out_id, pts in out_items]
                map_lines.append(f'{in_id}->[{", ".join(parts)}]')

            if map_lines:
                self.get_logger().info('id mapping (input->output by shared points): ' + ' | '.join(map_lines))
                self.get_logger().info(
                    f'split summary: input clusters split into multiple outputs = {split_count}')
            else:
                self.get_logger().warn('id mapping unavailable (all points are noise or empty clusters)')
        else:
            self.get_logger().warn(
                'input/output point count mismatch, skip id mapping '
                f'({in_stats.total_points} vs {out_stats.total_points})')

        self._input_cache.pop(key, None)
        self._output_cache.pop(key, None)
        self._done = True
        self.get_logger().info('single-shot compare finished. exiting...')
        rclpy.shutdown()



def main() -> None:
    input_topic = '/pointcloud/clustered'
    output_topic = '/pointcloud/clustered_split'
    max_ids_to_print = 0

    if len(sys.argv) >= 2:
        input_topic = sys.argv[1]
    if len(sys.argv) >= 3:
        output_topic = sys.argv[2]
    if len(sys.argv) >= 4:
        max_ids_to_print = int(sys.argv[3])

    rclpy.init()
    node = SplitterCompareNode(input_topic, output_topic, max_ids_to_print)

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
