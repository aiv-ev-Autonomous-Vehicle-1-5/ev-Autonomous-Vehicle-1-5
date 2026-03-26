"""
yolo_lane_cluster.launch.py — Yolo Lane Cluster 노드 런치 파일

차선 전처리 노드를 ComposableNode로 실행한다.
시드 기반 좌/우 판별 + 가상 차선 생성.

실행 명령:
  ros2 launch yolo_lane_cluster yolo_lane_cluster.launch.py
"""
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share   = get_package_share_directory('yolo_lane_cluster')
    params_file = os.path.join(pkg_share, 'config', 'yolo_lane_cluster.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['yolo_lane_cluster_node']['ros__parameters']

    container = ComposableNodeContainer(
        name='yolo_lane_cluster_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolo_lane_cluster',
                plugin='yolo_lane_cluster::YoloLaneClusterNode',
                name='yolo_lane_cluster_node',
                parameters=[params],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
