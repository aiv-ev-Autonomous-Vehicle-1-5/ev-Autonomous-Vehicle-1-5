"""Launch Velodyne perception pipeline for Gazebo simulation.

Gazebo's velodyne plugin publishes PointCloud2 directly to /velodyne_points,
so velodyne_driver and velodyne_transform are not needed.

Pipeline: /velodyne_points -> Patchwork++ -> DBSCAN(GPU) -> ClusterSplitter -> MakeBBox
"""

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # All perception parameters from lidar_launch config
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

    bbox_params_file = os.path.join(
        launch_share_dir, 'config', 'make_bbox', 'make_bbox_params.yaml')
    with open(bbox_params_file, 'r') as f:
        bbox_params = yaml.safe_load(f)['make_bbox']['ros__parameters']

    container = ComposableNodeContainer(
        name='velodyne_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. Patchwork++ - ground segmentation
            ComposableNode(
                package='patchworkpp',
                plugin='patchworkpp_ros::GroundSegmentationServer',
                name='patchworkpp_node',
                parameters=[patchworkpp_params],
                remappings=[
                    ('pointcloud_topic', 'velodyne_points'),
                ],
                ),  # intra-process disabled

            # 2. DBSCAN Clustering - GPU accelerated
            ComposableNode(
                package='dbscan_clustering',
                plugin='dbscan_clustering::DBSCANNode',
                name='dbscan_clustering',
                parameters=[dbscan_params],
                ),  # intra-process disabled

            # 3. Cluster Splitter - split over-merged cone clusters
            ComposableNode(
                package='cluster_splitter',
                plugin='cluster_splitter::ClusterSplitterNode',
                name='cluster_splitter',
                parameters=[splitter_params],
                ),  # intra-process disabled

            # 4. MakeBBox - PointCloud2 -> BBoxArray + MarkerArray
            ComposableNode(
                package='make_bbox',
                plugin='make_bbox::MakeBBoxNode',
                name='make_bbox',
                parameters=[bbox_params],
                ),  # intra-process disabled

        ],
        output='both',
    )

    return LaunchDescription([container])
