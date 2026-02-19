import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_clustering')
    param_file = os.path.join(pkg_share, 'config', 'clustering_params.yaml')

    container = ComposableNodeContainer(
        name='clustering_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_clustering',
                plugin='lidar_clustering::ClusteringComponent',
                name='clustering_component',
                parameters=[param_file],
                remappings=[
                    ('input', '/voxel_grid/output'),
                    ('output', '/clustering/nonground')
                ]
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
