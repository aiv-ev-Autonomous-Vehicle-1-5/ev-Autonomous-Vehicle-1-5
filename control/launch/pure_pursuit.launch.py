"""
pure_pursuit.launch.py — Pure Pursuit 제어 노드 런치 파일

상대좌표 Pure Pursuit 제어 노드를 실행한다.
파라미터는 config/pure_pursuit.yaml에서 로드한다.

실행 명령:
  ros2 launch pp_controller_cpp pure_pursuit.launch.py
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('pp_controller_cpp')
    params_file = os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')

    return LaunchDescription([
        Node(
            package='pp_controller_cpp',
            executable='pure_pursuit_relative_node',
            name='pure_pursuit_relative_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
