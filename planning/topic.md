# Planning Package — Topic 정리

## 입력 토픽 (Subscribe)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | Best Effort, depth=1 | 카메라 차선 인식 결과 (차선 경계점 배열) |
| `/perception/bboxes` | `ev_msgs/msg/BBoxArray` | Best Effort, depth=1 | LiDAR 장애물(콘/드럼) 바운딩 박스 |

---

## 출력 토픽 (Publish)

### Core 토픽 (항상 발행)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | Best Effort, depth=1 | 최종 후처리 경로 — 제어기(Pure Pursuit)가 구독 (초록색, 8cm) |
| `/planning/status` | `std_msgs/msg/String` | Best Effort, depth=1 | 플래너 상태: `OK`, `STALE`, `FAIL - not enough seeds`, `FAIL - no valid path`, `FAIL - too short valid path`, `WARNING - curvature exceeds r_min` |

### Debug 토픽 (Lazy Publishing — RViz2 구독 시에만 발행)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/debug/costmap` | `nav_msgs/msg/OccupancyGrid` | Reliable, depth=1 | Gaussian 코스트맵 시각화 |
| `/planning/debug/raw_path` | `visualization_msgs/msg/Marker` (POINTS) | Reliable, depth=1 | A* 원시 경로 — 후처리 전 (흰색, 6cm) |
| `/planning/debug/pruned_path` | `visualization_msgs/msg/Marker` (POINTS) | Reliable, depth=1 | prune 직후 경로 — smooth/curvature_clamp 전 (노란색, 10cm) |
| `/planning/debug/left_chain` | `nav_msgs/msg/Path` | Reliable, depth=1 | 왼쪽 backbone 체인 |
| `/planning/debug/right_chain` | `nav_msgs/msg/Path` | Reliable, depth=1 | 오른쪽 backbone 체인 |
| `/chaining/debug/left_branches` | `visualization_msgs/msg/MarkerArray` (LINE_STRIP) | Reliable, depth=1 | 왼쪽 branch 시각화 (연한 초록색) |
| `/chaining/debug/right_branches` | `visualization_msgs/msg/MarkerArray` (LINE_STRIP) | Reliable, depth=1 | 오른쪽 branch 시각화 (연한 분홍색) |
| `/chaining/debug/seeds` | `visualization_msgs/msg/MarkerArray` (SPHERE) | Reliable, depth=1 | 시드(시작점) 및 골(끝점) 마커 |
| `/planning/debug/local_goal` | `visualization_msgs/msg/MarkerArray` (SPHERE) | Reliable, depth=1 | A* 탐색 목표점 (노란색 구체, 30cm) |
| `/planning/debug/obstacle_wall` | `visualization_msgs/msg/MarkerArray` (CUBE_LIST) | Reliable, depth=1 | cost >= obstacle_cost 셀 (빨간색 큐브) |
| `/planning/debug/curvature` | `visualization_msgs/msg/MarkerArray` (SPHERE) | Reliable, depth=1 | 곡률 초과 지점 (노란~빨간 그라데이션) |

---

## 노드 정보

- **노드 이름**: `lc_planner_node` (ComposableNode)
- **실행 주기**: 10Hz (100ms wall timer)
- **좌표계**: `base_link` (모든 토픽)
- **설정 파일**: `config/chaining_costmap_ver.yaml`
