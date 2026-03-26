#!/usr/bin/env python3
"""rosbag에서 /perception/bboxes_marker 를 읽어 bbox 크기 통계를 출력한다.

사용법:
  python3 analyze_bbox_sizes.py <bag_dir>
  python3 analyze_bbox_sizes.py /home/aiv/ev-Autonomous-Vehicle-1-5/rosbag/whol_track1
"""

import sys
import statistics
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from visualization_msgs.msg import MarkerArray


def extract_bbox_sizes_from_markers(bag_path: str):
    """MarkerArray LINE_LIST에서 bbox 크기(size_x, size_y, size_z) 추출."""
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_opts, converter_opts)

    # 필터: bboxes_marker 토픽만
    topic_filter = reader.get_all_topics_and_types()
    target = '/perception/bboxes_marker'

    sizes = []  # list of (size_x, size_y, size_z)
    frame_count = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != target:
            continue

        msg = deserialize_message(data, MarkerArray)
        frame_count += 1

        for marker in msg.markers:
            if marker.ns != 'bboxes' or len(marker.points) < 2:
                continue

            xs = [p.x for p in marker.points]
            ys = [p.y for p in marker.points]
            zs = [p.z for p in marker.points]

            sx = max(xs) - min(xs)
            sy = max(ys) - min(ys)
            sz = max(zs) - min(zs)

            sizes.append((sx, sy, sz))

    return sizes, frame_count


def print_stats(sizes):
    if not sizes:
        print('bbox 데이터 없음')
        return

    sx_list = [s[0] for s in sizes]
    sy_list = [s[1] for s in sizes]
    sz_list = [s[2] for s in sizes]

    print(f'\n=== BBox 크기 분석 (총 {len(sizes)}개 bbox) ===\n')

    for name, data in [('size_x', sx_list), ('size_y', sy_list), ('size_z', sz_list)]:
        mn = min(data)
        mx = max(data)
        avg = statistics.mean(data)
        med = statistics.median(data)
        std = statistics.stdev(data) if len(data) > 1 else 0.0
        q25 = statistics.quantiles(data, n=4)[0] if len(data) >= 2 else avg
        q75 = statistics.quantiles(data, n=4)[2] if len(data) >= 2 else avg

        print(f'  {name}:')
        print(f'    min={mn:.4f}  max={mx:.4f}  mean={avg:.4f}  median={med:.4f}')
        print(f'    std={std:.4f}  Q25={q25:.4f}  Q75={q75:.4f}')
        print()

    # z_extent 분포 히스토그램 (텍스트)
    print('--- size_z 분포 히스토그램 ---')
    bins = [0.0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.80, 1.0, 999]
    labels = [
        '0.00-0.02', '0.02-0.05', '0.05-0.10', '0.10-0.15',
        '0.15-0.20', '0.20-0.30', '0.30-0.40', '0.40-0.50',
        '0.50-0.60', '0.60-0.80', '0.80-1.00', '1.00+     ',
    ]
    counts = [0] * len(labels)
    for sz in sz_list:
        for i in range(len(bins) - 1):
            if bins[i] <= sz < bins[i + 1]:
                counts[i] += 1
                break

    max_bar = 50
    max_count = max(counts) if max(counts) > 0 else 1
    for label, count in zip(labels, counts):
        bar = '#' * int(count / max_count * max_bar)
        pct = count / len(sz_list) * 100
        print(f'  {label}m | {bar:<{max_bar}} {count:5d} ({pct:5.1f}%)')

    # 납작한(평면 노이즈) vs 콘 높이 분류
    flat_threshold = 0.10
    flat_count = sum(1 for sz in sz_list if sz < flat_threshold)
    cone_like = sum(1 for sz in sz_list if 0.20 <= sz <= 0.90)
    print(f'\n--- 분류 ---')
    print(f'  납작한 클러스터 (z < {flat_threshold}m): {flat_count} ({flat_count/len(sz_list)*100:.1f}%)')
    print(f'  콘 유사 클러스터 (0.20~0.90m):          {cone_like} ({cone_like/len(sz_list)*100:.1f}%)')
    print(f'  기타:                                     {len(sz_list) - flat_count - cone_like}')


def main():
    if len(sys.argv) < 2:
        print(f'사용법: python3 {sys.argv[0]} <bag_directory>')
        sys.exit(1)

    bag_path = sys.argv[1]
    if not Path(bag_path).is_dir():
        print(f'ERROR: {bag_path} 디렉토리 없음')
        sys.exit(1)

    print(f'rosbag 분석 중: {bag_path}')
    sizes, frame_count = extract_bbox_sizes_from_markers(bag_path)
    print(f'프레임 수: {frame_count}')
    print_stats(sizes)


if __name__ == '__main__':
    main()
