"""Launch Velodyne full perception pipeline for real hardware.

Pipeline: Driver -> Transform -> Patchwork++ -> DBSCAN(GPU) -> MakeBBox
All nodes run as components in a single container for intra-process communication.
"""

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess



def generate_launch_description():
    # Velodyne driver parameters
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    # Velodyne transform parameters
    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')

    # All perception parameters from lidar_launch config
    launch_share_dir = ament_index_python.packages.get_package_share_directory('lidar_launch')

    patchworkpp_params_file = os.path.join(launch_share_dir, 'config', 'patchworkpp', 'patchworkpp_params.yaml')
    with open(patchworkpp_params_file, 'r') as f:
        patchworkpp_params = yaml.safe_load(f)['patchworkpp_node']['ros__parameters']

    dbscan_params_file = os.path.join(launch_share_dir, 'config', 'dbscan_clustering', 'dbscan_params.yaml')
    with open(dbscan_params_file, 'r') as f:
        dbscan_params = yaml.safe_load(f)['dbscan_clustering']['ros__parameters']

    bbox_params_file = os.path.join(
        launch_share_dir, 'config', 'make_bbox', 'make_bbox_params.yaml')
    with open(bbox_params_file, 'r') as f:
        bbox_params = yaml.safe_load(f)['make_bbox']['ros__parameters']
    
    debug_script = os.path.join(
        launch_share_dir,
        'launch',
        'perception',
        'debug_scripts',
        'inspect_bboxes.py',
    )
    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # 1. Velodyne driver - receives UDP packets
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[driver_params],
                    ),  # intra-process disabled

                # 2. Velodyne transform - converts packets to point cloud
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[convert_params],
                    ),  # intra-process disabled

                # 3. Patchwork++ - ground segmentation
                ComposableNode(
                    package='patchworkpp',
                    plugin='patchworkpp_ros::GroundSegmentationServer',
                    name='patchworkpp_node',
                    parameters=[patchworkpp_params],
                    remappings=[
                        ('pointcloud_topic', 'velodyne_points'),
                    ],
                    ),  # intra-process disabled

                # 4. DBSCAN Clustering - GPU accelerated
                ComposableNode(
                    package='dbscan_clustering',
                    plugin='dbscan_clustering::DBSCANNode',
                    name='dbscan_clustering',
                    parameters=[dbscan_params],
                    ),  # intra-process disabled

                # 5. make_bbox
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
    # base_link → velodyne 고정 TF
    static_tf_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        arguments=[
            "0.227", "0", "0.823",
            "0", "0", "0", "1",
            "base_link", "velodyne"
        ],
        output='screen'
    )

    return LaunchDescription([container, debug_bboxes, static_tf_velodyne])
