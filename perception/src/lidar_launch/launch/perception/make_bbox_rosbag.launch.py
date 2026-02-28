"""ROSbag 재생용 make_bbox 파이프라인.

make_bbox.launch.py와 동일하되, base_link → velodyne static TF를 포함한다.
rosbag에 /tf, /tf_static이 녹화되지 않은 경우를 위한 launch 파일.

사용법:
  ros2 launch lidar_launch make_bbox_rosbag.launch.py
  ros2 bag play ~/ev_ws/perception/rosbag/playground2_good/
"""

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_share_dir = ament_index_python.packages.get_package_share_directory('lidar_launch')

    # ── 파라미터 로드 ──
    patchworkpp_params_file = os.path.join(
        launch_share_dir, 'config', 'patchworkpp', 'patchworkpp_params.yaml')
    with open(patchworkpp_params_file, 'r') as f:
        patchworkpp_params = yaml.safe_load(f)['patchworkpp_node']['ros__parameters']

    dbscan_params_file = os.path.join(
        launch_share_dir, 'config', 'dbscan_clustering', 'dbscan_params.yaml')
    with open(dbscan_params_file, 'r') as f:
        dbscan_params = yaml.safe_load(f)['dbscan_clustering']['ros__parameters']

    bbox_params_file = os.path.join(
        launch_share_dir, 'config', 'make_bbox', 'make_bbox_params.yaml')
    with open(bbox_params_file, 'r') as f:
        bbox_params = yaml.safe_load(f)['make_bbox']['ros__parameters']

    # ── Static TF: base_link → velodyne (rosbag용) ──
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.90',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'velodyne',
        ],
    )

    # ── ComposableNode 컨테이너 (전체 파이프라인) ──
    container = ComposableNodeContainer(
        name='velodyne_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='patchworkpp',
                plugin='patchworkpp_ros::GroundSegmentationServer',
                name='patchworkpp_node',
                parameters=[patchworkpp_params],
                remappings=[
                    ('pointcloud_topic', 'velodyne_points'),
                ],
            ),
            ComposableNode(
                package='dbscan_clustering',
                plugin='dbscan_clustering::DBSCANNode',
                name='dbscan_clustering',
                parameters=[dbscan_params],
            ),
            ComposableNode(
                package='make_bbox',
                plugin='make_bbox::MakeBBoxNode',
                name='make_bbox',
                parameters=[bbox_params],
            ),
        ],
        output='both',
    )

    # ── 디버그: /perception/bboxes 검증 ──
    debug_script = os.path.join(
        launch_share_dir, 'launch', 'perception', 'debug_scripts', 'inspect_bboxes.py')
    debug_bboxes = ExecuteProcess(
        cmd=['python3', debug_script, '/perception/bboxes'],
        output='screen',
    )

    return LaunchDescription([tf_base_to_velodyne, container, debug_bboxes])
