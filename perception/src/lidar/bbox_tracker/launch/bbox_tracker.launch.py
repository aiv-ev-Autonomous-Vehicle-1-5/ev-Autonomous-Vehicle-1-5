import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_share = get_package_share_directory("lidar_launch")
    params = os.path.join(launch_share, "config", "bbox_tracker", "bbox_tracker_params.yaml")

    return LaunchDescription([
        Node(
            package="bbox_tracker",
            executable="bbox_tracker_node",
            name="bbox_tracker_node",
            output="screen",
            parameters=[params],
        )
    ])
