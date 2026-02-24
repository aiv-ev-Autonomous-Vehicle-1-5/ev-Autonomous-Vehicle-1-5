import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('track_planning')
    params_file = os.path.join(pkg_share, 'config', 'planning.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['local_planner_node']['ros__parameters']

    container = ComposableNodeContainer(
        name='planning_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='track_planning',
                plugin='track_planning::LocalPlannerNode',
                name='local_planner_node',
                parameters=[params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
