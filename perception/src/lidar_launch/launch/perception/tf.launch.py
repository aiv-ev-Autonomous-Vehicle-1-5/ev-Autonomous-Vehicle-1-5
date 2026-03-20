"""base_link → velodyne 고정 TF 발행.

Velodyne driver는 별도 launch(예: Gazebo plugin 등)에서 실행.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link → velodyne 고정 TF
    static_tf_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        arguments=[
            "0.224", "0", "0.485",
            "0", "0", "0", "1",
            "base_link", "velodyne"
        ],
        output='screen'
    )

    return LaunchDescription([static_tf_velodyne])
