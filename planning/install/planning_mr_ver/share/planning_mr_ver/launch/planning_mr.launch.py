import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share   = get_package_share_directory('planning_mr_ver')
    params_file = os.path.join(pkg_share, 'config', 'planning_mr.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['mr_planner_node']['ros__parameters']

    container = ComposableNodeContainer(
        name='planning_mr_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='planning_mr_ver',
                plugin='planning_mr_ver::MRPlannerNode',
                name='mr_planner_node',
                parameters=[params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
