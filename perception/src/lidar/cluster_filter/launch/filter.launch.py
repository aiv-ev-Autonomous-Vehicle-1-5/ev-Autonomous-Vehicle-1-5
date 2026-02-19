import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('cluster_filter')
    param_file = os.path.join(pkg_share, 'config', 'filter_params.yaml')

    container = ComposableNodeContainer(
        name='filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cluster_filter',
                plugin='cluster_filter::FilterComponent',
                name='filter_component',
                parameters=[param_file],
                remappings=[
                    ('input', '/clustering/nonground'),
                    ('output', '/clustering/filtered'),
                    ('cones', '/clustering/cones')
                ]
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
