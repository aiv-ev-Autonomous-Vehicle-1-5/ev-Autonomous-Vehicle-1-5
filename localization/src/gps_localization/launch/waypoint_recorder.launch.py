import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("gps_localization")

    params = os.path.join(pkg, "config", "waypoint_recorder.yaml")

    return LaunchDescription([
        Node(
            package="gps_localization",
            executable="waypoint_recorder_node",
            name="waypoint_recorder",
            output="screen",
            parameters=[
                params,
                {
                    "fix_topic": "/ublox_gps_node/fix",
                    # output_file을 지정하지 않으면 타임스탬프 기반 자동 생성
                    # "output_file": "/home/aiv/ev-Autonomous-Vehicle-1-5/localization/RDDF/my_track.csv",
                }
            ],
        )
    ])
