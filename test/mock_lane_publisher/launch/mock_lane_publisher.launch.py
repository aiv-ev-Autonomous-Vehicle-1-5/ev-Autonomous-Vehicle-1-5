import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mock_lane_publisher'),
        'config',
        'params.yaml',
    )

    return LaunchDescription([
        Node(
            package='mock_lane_publisher',
            executable='mock_lane_node',
            name='mock_lane_publisher',
            output='screen',
            parameters=[config],
        ),
    ])
