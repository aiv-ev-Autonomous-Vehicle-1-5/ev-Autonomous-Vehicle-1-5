import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_tracking')
    param_file = os.path.join(pkg_share, 'config', 'tracking_params.yaml')

    # Since tracking_params.yaml has a '/**' key, we might need to load it and extract parameters
    # or rely on rclcpp to handle it. The original launch file did this:
    # with open(tracking_params_file, 'r') as f:
    #     tracking_params = yaml.safe_load(f)['/**']['ros__parameters']
    
    # We will do the same to be safe.
    with open(param_file, 'r') as f:
        tracking_params = yaml.safe_load(f)['/**']['ros__parameters']

    container = ComposableNodeContainer(
        name='tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_tracking',
                plugin='lidar_tracking::TrackingNode',
                name='tracking_node',
                parameters=[tracking_params],
                remappings=[
                    ('input', '/lidar/cones_detected'),
                    ('output', '/lidar/cones_tracked')
                ]
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
