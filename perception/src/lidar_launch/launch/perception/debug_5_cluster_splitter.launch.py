"""Debug Stage 5: Patchwork++ + DBSCAN + ClusterSplitter.

Use this launch to validate merged-cluster split behavior quickly in simulation.
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

    patchworkpp_params_file = os.path.join(launch_share_dir, 'config', 'patchworkpp', 'patchworkpp_params.yaml')
    with open(patchworkpp_params_file, 'r') as f:
        patchworkpp_params = yaml.safe_load(f)['patchworkpp_node']['ros__parameters']

    dbscan_params_file = os.path.join(launch_share_dir, 'config', 'dbscan_clustering', 'dbscan_params.yaml')
    with open(dbscan_params_file, 'r') as f:
        dbscan_params = yaml.safe_load(f)['dbscan_clustering']['ros__parameters']

    splitter_params_file = os.path.join(
        launch_share_dir, 'config', 'cluster_splitter', 'cluster_splitter_params.yaml')
    with open(splitter_params_file, 'r') as f:
        splitter_params = yaml.safe_load(f)['cluster_splitter']['ros__parameters']

    debug_script = os.path.join(
        launch_share_dir,
        'launch',
        'perception',
        'debug_scripts',
        'compare_splitter_clusters.py',
    )

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
                package='cluster_splitter',
                plugin='cluster_splitter::ClusterSplitterNode',
                name='cluster_splitter',
                parameters=[splitter_params],
            ),
        ],
        output='both',
    )

    debug_counter = ExecuteProcess(
        cmd=['python3', debug_script, '/pointcloud/clustered_split'],
        output='screen',
    )



    return LaunchDescription([container, debug_counter])
