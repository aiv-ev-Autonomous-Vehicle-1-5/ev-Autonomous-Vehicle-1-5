"""Launch Velodyne with Ground Segmentation and CropBox filter in a composable container."""

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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

    # CropBox parameters
    cropbox_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_cropbox')
    cropbox_params_file = os.path.join(cropbox_share_dir, 'config', 'default_cropbox_params.yaml')
    with open(cropbox_params_file, 'r') as f:
        cropbox_params = yaml.safe_load(f)['cropbox_component']['ros__parameters']

    # Patchwork++ parameters
    patchworkpp_share_dir = ament_index_python.packages.get_package_share_directory('patchworkpp')
    patchworkpp_params_file = os.path.join(patchworkpp_share_dir, 'config', 'patchworkpp_params.yaml')
    with open(patchworkpp_params_file, 'r') as f:
        patchworkpp_params = yaml.safe_load(f)['patchworkpp_node']['ros__parameters']

    # VoxelGrid parameters
    voxel_grid_share_dir = ament_index_python.packages.get_package_share_directory('lidar_voxel_grid')
    voxel_grid_params_file = os.path.join(voxel_grid_share_dir, 'config', 'voxel_grid_params.yaml')
    with open(voxel_grid_params_file, 'r') as f:
        voxel_grid_params = yaml.safe_load(f)['voxel_grid_component']['ros__parameters']

    # DBSCAN Clustering parameters
    clustering_share_dir = ament_index_python.packages.get_package_share_directory('lidar_clustering')
    clustering_params_file = os.path.join(clustering_share_dir, 'config', 'clustering_params.yaml')
    with open(clustering_params_file, 'r') as f:
        clustering_params = yaml.safe_load(f)['clustering_component']['ros__parameters']

    # Cluster Filter parameters
    cluster_filter_share_dir = ament_index_python.packages.get_package_share_directory('cluster_filter')
    cluster_filter_params_file = os.path.join(cluster_filter_share_dir, 'config', 'filter_params.yaml')
    with open(cluster_filter_params_file, 'r') as f:
        cluster_filter_params = yaml.safe_load(f)['filter_component']['ros__parameters']

    # Tracking parameters
    tracking_share_dir = ament_index_python.packages.get_package_share_directory('lidar_tracking')
    tracking_params_file = os.path.join(tracking_share_dir, 'config', 'tracking_params.yaml')
    # Tracking params are root level
    with open(tracking_params_file, 'r') as f:
        tracking_params = yaml.safe_load(f)['/**']['ros__parameters']

    # Pipeline: Driver -> Transform -> CropBox -> Patchwork++ -> VoxelGrid -> DBSCAN
    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # 1. Velodyne driver - receives UDP packets
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[driver_params]),

                # 2. Velodyne transform - converts packets to point cloud
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[convert_params]),

                # 3. CropBox filter - ROI filtering (preserves ring/time fields)
                ComposableNode(
                    package='velodyne_cropbox',
                    plugin='velodyne_cropbox::CropBoxComponent',
                    name='cropbox_component',
                    parameters=[cropbox_params],
                    remappings=[
                        ('input', 'velodyne_points'),
                        ('output', 'velodyne_points_cropped')
                    ]),

                # 4. Patchwork++ - ground segmentation
                ComposableNode(
                    package='patchworkpp',
                    plugin='patchworkpp_ros::GroundSegmentationServer',
                    name='patchworkpp_node',
                    parameters=[patchworkpp_params],
                    remappings=[
                        ('pointcloud_topic', 'velodyne_points_cropped'),
                    ]),

                # 5. VoxelGrid Downsampling
                ComposableNode(
                    package='lidar_voxel_grid',
                    plugin='lidar_voxel_grid::VoxelGridComponent',
                    name='voxel_grid_component',
                    parameters=[voxel_grid_params],
                    remappings=[
                        ('input', '/patchworkpp/nonground'),
                        ('output', '/voxel_grid/output')
                    ]),

                # 6. DBSCAN Clustering - obstacle grouping
                ComposableNode(
                    package='lidar_clustering',
                    plugin='lidar_clustering::ClusteringComponent',
                    name='clustering_component',
                    parameters=[clustering_params],
                    remappings=[
                        ('input', '/voxel_grid/output'),
                        ('output', '/clustering/nonground')
                    ]),

                # 7. Cluster Filter - noise/wall/floor remnant removal, RGB coloring
                ComposableNode(
                    package='cluster_filter',
                    plugin='cluster_filter::FilterComponent',
                    name='filter_component',
                    parameters=[cluster_filter_params],
                    remappings=[
                        ('input', '/clustering/nonground'),
                        ('output', '/clustering/filtered'),
                        ('cones', '/clustering/cones'),
                    ]),

                # 8. Tracking Node - L-ByteTrack
                ComposableNode(
                    package='lidar_tracking',
                    plugin='lidar_tracking::TrackingNode',
                    name='tracking_node',
                    parameters=[tracking_params],
                    remappings=[
                        ('input', '/lidar/cones_detected'),
                        ('output', '/lidar/cones_tracked'),
                    ]),

            ],
            output='both',
    )

    return LaunchDescription([container])
