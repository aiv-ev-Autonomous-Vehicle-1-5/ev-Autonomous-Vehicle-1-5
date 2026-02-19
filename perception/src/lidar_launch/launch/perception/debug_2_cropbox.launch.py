"""Debug Stage 2: Driver + Transform + CropBox.

Verify ROI-filtered output on /velodyne_points_cropped.
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

    # CropBox parameters
    cropbox_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_cropbox')
    cropbox_params_file = os.path.join(cropbox_share_dir, 'config', 'default_cropbox_params.yaml')
    with open(cropbox_params_file, 'r') as f:
        cropbox_params = yaml.safe_load(f)['cropbox_component']['ros__parameters']

    # Stage 2: Driver -> Transform -> CropBox -> /velodyne_points_cropped
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

                # 3. CropBox filter - ROI filtering
                ComposableNode(
                    package='velodyne_cropbox',
                    plugin='velodyne_cropbox::CropBoxComponent',
                    name='cropbox_component',
                    parameters=[cropbox_params],
                    remappings=[
                        ('input', 'velodyne_points'),
                        ('output', 'velodyne_points_cropped')
                    ]),
            ],
            output='both',
    )

    return LaunchDescription([container])
