"""Debug Stage 4: Driver + Transform + Patchwork++ + VoxelGrid.

Verify downsampled nonground output on /voxel_grid/output.
"""

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

    # All perception parameters from lidar_launch config
    launch_share_dir = ament_index_python.packages.get_package_share_directory('lidar_launch')

    patchworkpp_params_file = os.path.join(launch_share_dir, 'config', 'patchworkpp', 'patchworkpp_params.yaml')
    with open(patchworkpp_params_file, 'r') as f:
        patchworkpp_params = yaml.safe_load(f)['patchworkpp_node']['ros__parameters']

    voxel_grid_params_file = os.path.join(launch_share_dir, 'config', 'lidar_voxel_grid', 'voxel_grid_params.yaml')
    with open(voxel_grid_params_file, 'r') as f:
        voxel_grid_params = yaml.safe_load(f)['voxel_grid_component']['ros__parameters']

    # Stage 4: Driver -> Transform -> Patchwork++ -> VoxelGrid
    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # # 1. Velodyne driver - receives UDP packets
                # ComposableNode(
                #     package='velodyne_driver',
                #     plugin='velodyne_driver::VelodyneDriver',
                #     name='velodyne_driver_node',
                #     parameters=[driver_params]),

                # # 2. Velodyne transform - converts packets to point cloud
                # ComposableNode(
                #     package='velodyne_pointcloud',
                #     plugin='velodyne_pointcloud::Transform',
                #     name='velodyne_transform_node',
                #     parameters=[convert_params]),

                # 3. Patchwork++ - ground segmentation
                ComposableNode(
                    package='patchworkpp',
                    plugin='patchworkpp_ros::GroundSegmentationServer',
                    name='patchworkpp_node',
                    parameters=[patchworkpp_params],
                    remappings=[
                        ('pointcloud_topic', 'velodyne_points'),
                    ]),

                # 4. VoxelGrid Downsampling
                ComposableNode(
                    package='lidar_voxel_grid',
                    plugin='lidar_voxel_grid::VoxelGridComponent',
                    name='voxel_grid_component',
                    parameters=[voxel_grid_params],
                    remappings=[
                        ('input', '/patchworkpp/nonground'),
                        ('output', '/voxel_grid/output')
                    ]),
            ],
            output='both',
    )

    return LaunchDescription([container])
