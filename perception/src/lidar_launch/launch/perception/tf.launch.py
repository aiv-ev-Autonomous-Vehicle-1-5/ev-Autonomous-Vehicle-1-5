"""base_link → velodyne / camera1 고정 TF 발행.

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
            "0.287", "0", "0.73", # 원래 0.287
            "0", "0", "0", "1",
            "base_link", "velodyne"
        ],
        output='screen'
    )

    # base_link → camera1 고정 TF
    # camera1_tf.yaml의 rotation_matrix → quaternion 변환 결과
    static_tf_camera1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera1',
        arguments=[
            "0.267", "0.0", "1.264",
            "-0.3415425548", "0.7959640708", "-0.4646470184", "0.1841005961",
            "base_link", "camera1"
        ],
        output='screen'
    )

    return LaunchDescription([static_tf_velodyne, static_tf_camera1])
