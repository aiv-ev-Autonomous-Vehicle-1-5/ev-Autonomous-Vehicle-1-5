import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- config paths ---
    fusion_share = get_package_share_directory("localization_fusion")
    fusion_params = os.path.join(fusion_share, "config", "localization_fuser.yaml")

    # gps_localization은 현재 코드가 파라미터 파일을 반드시 쓰는 구조는 아니지만,
    # (필요하면 gps_localization 쪽 yaml을 나중에 추가 가능)
    # gps_share = get_package_share_directory("gps_localization")
    # gps_params = os.path.join(gps_share, "config", "utm_localizer.yaml")

    # --- Nodes ---
    utm_localizer = Node(
        package="gps_localization",
        executable="utm_localizer_node",
        name="utm_localizer",
        output="screen",
        # ublox fix를 utm_localizer가 쓰는 fix_topic으로 맞추는 방식
        # 지금 utm_localizer는 fix_topic 파라미터를 받는 버전이므로 파라미터로 지정 가능.
        # (네 코드에 declare/get fix_topic 있으니 OK)
        parameters=[{
            "fix_topic": "/ublox_gps_node/fix",
            # waypoint/path를 utm_localizer에서 만들고 싶으면 아래도 지정(선택)
            # "waypoint_file": "/home/kimsohee/.../track.csv",
        }],
        # utm_localizer 출력 pose 토픽은 /gps_pose로 고정(너 코드가 이미 그렇게 publish 중)
        # 만약 코드가 /local_pose로 되어 있다면 여기서 remap:
        # remappings=[("/local_pose", "/gps_pose")],
    )

    localization_fuser = Node(
        package="localization_fusion",
        executable="localization_fuser",
        name="localization_fuser",
        output="screen",
        parameters=[fusion_params],
        # 필요하면 여기서도 remap 가능:
        # remappings=[("/gps_pose", "/gps_pose"), ("/erp42/odometry_wheel", "/erp42/odometry_wheel")]
    )

    return LaunchDescription([
        utm_localizer,
        localization_fuser,
    ])