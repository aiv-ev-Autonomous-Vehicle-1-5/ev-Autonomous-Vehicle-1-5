from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("gps_localization")
    params = os.path.join(pkg, "config", "utm_localizer.yaml")

    return LaunchDescription([
        Node(
            package="gps_localization",
            executable="utm_localizer",
            name="utm_localizer",
            output="screen",
            parameters=[params],
        )
    ])