import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory('track_bringup')
    params_file = os.path.join(bringup_share, 'config', 'costmap_builder_params.yaml')

    container = ComposableNodeContainer(
        name='planning_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='track_planning',
                plugin='track_planning::CostmapBuilderNode',
                name='costmap_builder',
                parameters=[params_file],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
