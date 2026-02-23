import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_voxel_grid')
    param_file = os.path.join(pkg_share, 'config', 'voxel_grid_params.yaml')

    container = ComposableNodeContainer(
        name='voxel_grid_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_voxel_grid',
                plugin='lidar_voxel_grid::VoxelGridComponent',
                name='voxel_grid_component',
                parameters=[param_file],
                remappings=[
                    ('input', '/patchworkpp/nonground'),
                    ('output', '/voxel_grid/output')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
