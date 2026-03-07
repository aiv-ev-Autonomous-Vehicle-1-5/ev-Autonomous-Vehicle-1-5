import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory("gps_localization")

    params = os.path.join(pkg, "config", "utm_localizer.yaml")
    waypoint = os.path.join(pkg, "waypoints", "track.csv")

    return LaunchDescription([
        Node(
            package="gps_localization",
            executable="utm_localizer",
            name="utm_localizer",
            output="screen",
            parameters=[
                params,
                {
                    # 토픽명도 여기서 한번에 고정/변경 가능
                    "fix_topic": "/ublox_gps_node/fix",
                    # waypoint_file은 패키지 share 기준으로 자동 세팅
                    "waypoint_file": waypoint,
                }
            ],
        )
    ])