"""
planning_lc.launch.py — LC Planner 런치 파일

DirectionChainer + Magnetic Resistance costmap 플래너를 ComposableNode로 실행한다.

실행 명령:
  ros2 launch chaining_CDT planning_lc.launch.py
"""
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share   = get_package_share_directory('chaining_CDT')
    params_file = os.path.join(pkg_share, 'config', 'chaining_CDT.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['lc_planner_node']['ros__parameters']

    container = ComposableNodeContainer(
        name='chaining_CDT_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='chaining_CDT',
                plugin='chaining_CDT::LCPlannerNode',
                name='lc_planner_node',
                parameters=[params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
