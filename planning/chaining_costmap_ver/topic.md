# chaining_costmap_ver 패키지 토픽 정리

## 구독 토픽 (Subscriptions)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | 카메라 차선 인식 결과 |
| `/perception/bboxes` | `ev_msgs/msg/BBoxArray` | BestEffort, depth=1 | LiDAR 장애물 바운딩 박스 |

## 발행 토픽 (Publications)

### 코어 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | BestEffort | 최종 후처리 경로 (초록 포인트) |
| `/planning/status` | `std_msgs/msg/String` | BestEffort | 플래너 상태 (OK/WARNING/FAIL/STALE) |

### 디버그 토픽 (Lazy)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/debug/costmap` | `nav_msgs/msg/OccupancyGrid` | Reliable | 가우시안 코스트맵 시각화 |
| `/planning/debug/raw_path` | `visualization_msgs/msg/Marker` (POINTS) | Reliable | A* 원시 경로 (흰색) |
| `/planning/debug/pruned_path` | `visualization_msgs/msg/Marker` (POINTS) | Reliable | 프루닝 후 경로 (노란색) |
| `/planning/debug/left_chain` | `nav_msgs/msg/Path` | Reliable | 좌측 백본 체인 |
| `/planning/debug/right_chain` | `nav_msgs/msg/Path` | Reliable | 우측 백본 체인 |
| `/chaining/debug/seeds` | `visualization_msgs/msg/MarkerArray` | Reliable | 시드 및 골 마커 |
| `/planning/debug/local_goal` | `visualization_msgs/msg/MarkerArray` | Reliable | A* 골 포인트 (노란 구) |
| `/planning/debug/obstacle_wall` | `visualization_msgs/msg/MarkerArray` | Reliable | 장애물 코스트 셀 (빨간 마커) |
| `/planning/debug/curvature` | `visualization_msgs/msg/MarkerArray` | Reliable | 곡률 초과 지점 (노란 구) |
| `/planning/debug/lane_points` | `visualization_msgs/msg/MarkerArray` | Reliable | 수신된 차선 포인트 (마젠타 구) |
| `/planning/debug/center_line` | `visualization_msgs/msg/Marker` (POINTS) | Reliable | 중앙선 유도 포인트 (노란 포인트) |
