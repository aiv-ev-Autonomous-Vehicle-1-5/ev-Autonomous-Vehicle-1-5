import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory("localization_fusion")
    params = os.path.join(pkg, "config", "localization_fuser.yaml")

    return LaunchDescription([
        Node(
            package="localization_fusion",
            executable="localization_fuser",
            name="localization_fuser",
            output="screen",
            parameters=[params],
        )
    ])