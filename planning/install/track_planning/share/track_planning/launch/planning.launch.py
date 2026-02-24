# ============================================================
# planning.launch.py
#
# 역할:
#   - track_planning 패키지의 LocalPlannerNode를
#     ComposableNodeContainer 패턴으로 실행한다.
#
# ComposableNodeContainer 패턴 설명:
#   - 일반 Node 실행: 각 노드가 별도의 프로세스로 실행됨 (Node() 방식)
#   - ComposableNode 실행: 여러 노드가 하나의 컨테이너 프로세스 안에서 실행됨
#     → 같은 컨테이너 안의 노드끼리 intra-process 통신 가능
#     → intra-process = 프로세스 내부 통신 = 네트워크/DDS를 거치지 않음
#     → use_intra_process_comms: True 설정 시 UniquePtr 소유권 이전으로 Zero-copy 달성
#
# 실행 흐름:
#   1) ament_index로 패키지 공유 디렉토리 경로 획득
#   2) YAML 파일에서 파라미터 로드
#   3) ComposableNodeContainer 안에 LocalPlannerNode 등록
#   4) LaunchDescription 반환 → ros2 launch가 이를 실행
# ============================================================

import os
import yaml

# launch: ROS 2 launch 시스템의 핵심 모듈
from launch import LaunchDescription

# ComposableNodeContainer: 여러 ComposableNode를 담는 컨테이너 프로세스 액션
from launch_ros.actions import ComposableNodeContainer

# ComposableNode: 컨테이너 안에 로드될 개별 노드 설명(descriptor)
from launch_ros.descriptions import ComposableNode

# get_package_share_directory: 설치된 패키지의 share 디렉토리 절대 경로 반환
# 예: /home/aiv/ev_ws/install/track_planning/share/track_planning/
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ============================================================
    # (1) 파라미터 파일 경로 획득
    #
    # get_package_share_directory('track_planning'):
    #   colcon 빌드 후 install/ 디렉토리 안의 패키지 share 경로 반환
    #   → CMakeLists.txt의 install(DIRECTORY config/ DESTINATION share/...) 로 설치된 파일들이 여기 있음
    #
    # params_file: planning.yaml의 절대 경로
    #   → YAML에는 local_planner_node 노드의 ros__parameters가 정의되어 있음
    # ============================================================
    pkg_share   = get_package_share_directory('track_planning')
    params_file = os.path.join(pkg_share, 'config', 'planning.yaml')

    # ============================================================
    # (2) YAML 파라미터 파일 로드
    #
    # yaml.safe_load(f): YAML 파일을 Python dict로 파싱
    # ['local_planner_node']['ros__parameters']:
    #   YAML 구조: local_planner_node: { ros__parameters: { ... } }
    #   에서 실제 파라미터 dict만 추출
    #
    # 결과 예시:
    #   params = {
    #     'timeouts': {'odom_ms': 200.0, 'perception_ms': 500.0},
    #     'corridor': {'min_width': 0.5, ...},
    #     ...
    #   }
    #
    # 이 dict를 ComposableNode의 parameters=[params]에 전달하면
    # ROS 2가 노드 초기화 시 파라미터 서버에 자동으로 로드해준다.
    # ============================================================
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['local_planner_node']['ros__parameters']

    # ============================================================
    # (3) ComposableNodeContainer 생성
    #
    # ComposableNodeContainer 필드 설명:
    #   name:       컨테이너 프로세스의 ROS 2 노드 이름
    #               → ros2 node list에 /planning_container 로 표시됨
    #   namespace:  노드 네임스페이스 (빈 문자열 = 전역 네임스페이스)
    #   package:    컨테이너 실행파일이 속한 패키지
    #               → rclcpp_components 패키지 사용 (ROS 2 기본 제공)
    #   executable: 컨테이너 실행파일 이름
    #               → component_container: 기본 싱글스레드 컨테이너
    #               → component_container_mt: 멀티스레드 컨테이너 (옵션)
    #   output:     로그 출력 대상
    #               → 'both': 터미널(screen)과 로그 파일 모두에 출력
    # ============================================================
    container = ComposableNodeContainer(
        name='planning_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        # --------------------------------------------------------
        # composable_node_descriptions: 컨테이너에 로드할 노드 목록
        # 여러 ComposableNode를 리스트로 추가하면 같은 프로세스에서 실행됨
        # --------------------------------------------------------
        composable_node_descriptions=[
            ComposableNode(
                # 노드가 속한 패키지 이름
                package='track_planning',

                # plugin: RCLCPP_COMPONENTS_REGISTER_NODE로 등록한 클래스의 완전한 이름
                # → 형식: '네임스페이스::클래스명'
                # → 컨테이너가 런타임에 이 이름으로 factory 함수를 찾아 노드 생성
                plugin='track_planning::LocalPlannerNode',

                # name: ROS 2에서 이 노드 인스턴스의 이름
                # → ros2 node list에 /local_planner_node 로 표시됨
                name='local_planner_node',

                # parameters: 노드 초기화 시 파라미터 서버에 로드할 dict
                # → YAML에서 로드한 params dict를 리스트로 감싸서 전달
                # → 노드 생성자에서 params_.load(this)로 읽어 사용
                parameters=[params],

                # extra_arguments: NodeOptions에 전달할 추가 설정
                # use_intra_process_comms: True 의 의미:
                #   - 같은 컨테이너 안의 노드끼리 메시지를 주고받을 때
                #     DDS/네트워크 스택을 우회하여 직접 메모리 포인터를 이전
                #   - Subscription 콜백이 UniquePtr를 받아야 효과 발생
                #   - Publisher.publish(std::move(msg)) 형태로 소유권 이전
                #   - 결과: 메모리 복사 0회 (Zero-copy) → 레이턴시 최소화
                #   - 제약: 같은 컨테이너 안의 노드 간에만 적용됨
                #           다른 프로세스 구독자에게는 일반 DDS 통신으로 fallback
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',  # 터미널과 로그 파일 모두에 출력
    )

    # ============================================================
    # (4) LaunchDescription 반환
    #
    # LaunchDescription: ros2 launch가 실행할 액션(Action) 목록을 담는 컨테이너
    # [container]를 리스트로 전달하면 launch 시스템이 컨테이너 프로세스를 시작함
    #
    # 실행 방법:
    #   ros2 launch track_planning planning.launch.py
    # ============================================================
    return LaunchDescription([container])
