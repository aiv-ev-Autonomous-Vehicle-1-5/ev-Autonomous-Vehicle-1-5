"""
planning_mr.launch.py — MR Planner 런치 파일

이 런치 파일은 MRPlannerNode를 ComposableNode로 실행한다.

실행 구조:
  ComposableNodeContainer (component_container)
    └── MRPlannerNode (ComposableNode)
        - use_intra_process_comms: True → Zero-copy 메시지 전달 활성화
        - parameters: planning_mr.yaml에서 로드

실행 명령:
  ros2 launch planning_mr_ver planning_mr.launch.py

Intra-process 통신 장점:
  - 같은 container 내 노드 간 메시지 전달 시 메모리 복사 없음 (Zero-copy)
  - LiDAR/카메라 데이터 같은 대용량 메시지에서 지연 시간 대폭 감소
"""
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 이 패키지의 share 디렉토리 경로 (install/planning_mr_ver/share/...)
    pkg_share   = get_package_share_directory('planning_mr_ver')
    # YAML 파라미터 파일 경로
    params_file = os.path.join(pkg_share, 'config', 'planning_mr.yaml')

    # YAML 파일을 직접 파싱하여 파라미터 딕셔너리로 변환
    # 'mr_planner_node' → 'ros__parameters' 하위의 모든 파라미터를 추출
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['mr_planner_node']['ros__parameters']

    # ComposableNodeContainer: rclcpp_components의 component_container 프로세스
    # 이 컨테이너 안에 여러 ComposableNode를 넣을 수 있다.
    container = ComposableNodeContainer(
        name='planning_mr_container',  # 컨테이너 노드 이름
        namespace='',                   # 네임스페이스 (비어있음 = 루트)
        package='rclcpp_components',    # component_container가 있는 패키지
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='planning_mr_ver',                    # 이 패키지
                plugin='planning_mr_ver::MRPlannerNode',      # C++ 클래스 이름
                name='mr_planner_node',                       # ROS 2 노드 이름
                parameters=[params],                          # YAML 파라미터
                extra_arguments=[{'use_intra_process_comms': True}],  # Zero-copy 활성화
            ),
        ],
        output='both',  # stdout + stderr 모두 출력
    )

    return LaunchDescription([container])
