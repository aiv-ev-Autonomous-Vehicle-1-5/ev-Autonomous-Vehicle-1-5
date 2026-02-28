# EV Autonomous Driving Workspace

## Project Overview
- 제5회 국제 대학생 EV 자율주행 경진대회 준비
- LiDAR + Camera 기반 자율주행 시스템
- ROS 2 Humble, C++17

## Workspace Structure
```
~/ev_ws/
├── ev_msgs/         # 커스텀 메시지 (BBox, BBoxArray 등)
├── perception/      # LiDAR 파이프라인 (Patchwork++, DBSCAN, MakeBBox)
├── planning/        # 경로 계획 (DirectionChainer + CDT Centerline)
├── erp42_ros/       # ERP42 차량 인터페이스
├── carsa_gazebo/    # Gazebo 시뮬레이션
└── agent_comm/      # Multi-Agent 협업 상태 파일
```

## Build Commands
**colcon build 시 반드시 `--symlink-install` 플래그를 사용할 것.**
```bash
# 개별 패키지 빌드
cd ~/ev_ws/perception && colcon build --symlink-install --packages-select <패키지명>
cd ~/ev_ws/planning && colcon build --symlink-install --packages-select chaining_CDT

# 전체 빌드
cd ~/ev_ws/perception && colcon build --symlink-install
cd ~/ev_ws/planning && colcon build --symlink-install
```

## Key Packages
| Package | Location | Description |
|---------|----------|-------------|
| patchworkpp | perception/src/lidar/ | 지면 분리 |
| dbscan_clustering | perception/src/lidar/ | 클러스터링 |
| make_bbox | perception/src/lidar/ | BBox 생성 + 크기 필터 |
| lidar_launch | perception/src/lidar_launch/ | launch + config 파일 |
| chaining_CDT | planning/src/chaining_CDT/ | 체이닝 + CDT 경로 생성 |
| ev_msgs | ev_msgs/ | BBox.msg, BBoxArray.msg |

## Architecture (Data Flow)
```
velodyne_points (PointCloud2)
  → Patchwork++ (지면 분리)
  → DBSCAN (클러스터링)
  → MakeBBox (BBox 생성 + 크기 필터)
  → /perception/bboxes (BBoxArray)
  → DirectionChainer (좌/우 체인 생성)
  → CDT Centerline (외심 기반 중심선 추출)
  → PostProcessor (smooth + resample)
  → /planning/path (Path)
```

## Sensors
- **LiDAR**: VLP-16 (192.168.1.201), frame_id: velodyne
- **Camera**: USB camera (usb_cam), frame_id: camera_link
- **TF**: base_link → velodyne (z: 0.90m), base_link → camera_link (TBD)

## Competition Constraints
- 차선 구간 GPS 사용 금지 (카메라 + LiDAR만)
- 차로 폭 1.5m, 흰색 테이프 10cm
- 라바콘: 직경 500mm, 높이 840mm (PE 드럼)
- 교통 콘 최소 3개 연속 배치

## Coding Conventions
- ComposableNode 패턴 사용 (intra-process communication 대비)
- 파라미터는 yaml 파일로 관리 (하드코딩 금지)
- QoS: 센서 입력은 SensorDataQoS (Best Effort), 내부 통신은 용도에 따라 선택
