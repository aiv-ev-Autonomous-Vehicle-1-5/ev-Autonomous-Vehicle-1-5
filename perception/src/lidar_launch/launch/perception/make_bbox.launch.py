"""Debug Stage 6: Patchwork++ + DBSCAN + MakeBBox 출력 검증.

make_bbox가 생성하는 /perception/bboxes (ev_msgs/BBoxArray)와
/perception/bboxes_marker (MarkerArray)를 검증한다.
디버그 스크립트가 첫 BBoxArray 프레임을 수신하면 bbox 정보를 출력하고 종료한다.
"""

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
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

    # ── 디버그 스크립트 (BBoxArray 검증) ──
    debug_script = os.path.join(
        launch_share_dir,
        'launch',
        'perception',
        'debug_scripts',
        'inspect_bboxes.py',
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
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='dbscan_clustering',
                plugin='dbscan_clustering::DBSCANNode',
                name='dbscan_clustering',
                parameters=[dbscan_params],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='make_bbox',
                plugin='make_bbox::MakeBBoxNode',
                name='make_bbox',
                parameters=[bbox_params],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    # ── 디버그: /perception/bboxes 검증 ──
    debug_bboxes = ExecuteProcess(
        cmd=['python3', debug_script, '/perception/bboxes'],
        output='screen',
    )

    return LaunchDescription([container, debug_bboxes])
