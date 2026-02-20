import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('cluster_splitter')
    params_file = os.path.join(pkg_share, 'config', 'cluster_splitter_params.yaml')

    container = ComposableNodeContainer(
        name='cluster_splitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cluster_splitter',
                plugin='cluster_splitter::ClusterSplitterNode',
                name='cluster_splitter',
                parameters=[params_file],
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
