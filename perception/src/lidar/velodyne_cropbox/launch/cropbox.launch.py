import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('velodyne_cropbox')
    param_file = os.path.join(pkg_share, 'config', 'default_cropbox_params.yaml')

    container = ComposableNodeContainer(
        name='cropbox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velodyne_cropbox',
                plugin='velodyne_cropbox::CropBoxComponent',
                name='cropbox_component',
                parameters=[param_file],
                remappings=[
                    ('input', 'velodyne_points'),
                    ('output', 'velodyne_points_cropped')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
