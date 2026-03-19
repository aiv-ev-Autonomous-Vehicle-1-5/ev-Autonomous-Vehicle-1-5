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
| `/planning/status` | `std_msgs/msg/String` | Best Effort, depth=1 | 플래너 상태 (아래 표 참고) |

#### 플래너 상태 (`/planning/status`) 상세

| 상태 | 발생 시점 | 발생 조건 | 경로 발행 |
|---|---|---|---|
| `OK` | Stage 6 (Safety Check) | 모든 검사 통과. 경로 유효, 곡률 정상 | 정상 경로 발행 |
| `STALE` | Stage 0 (Stale Gate) | perception 데이터(bbox/lane) 마지막 수신 시각이 `perception_ms`(300ms) 초과 — 센서 입력 없음 또는 지연 | 발행 안 함 (즉시 return) |
| `FAIL - not enough seeds` | Stage 2.5 (Seed Gate) | DirectionChainer가 좌/우 seed를 모두 찾지 못함 → 좌/우 backbone이 둘 다 비어있음 (`dc_result.valid == false`). 차선/콘이 전혀 감지되지 않거나 seed 조건(`side_seed_y`)을 만족하는 점이 없는 경우 | 빈 경로 발행 (즉시 return, Stage 3~7 스킵) |
| `FAIL - no valid path` | Stage 6 (Safety Check) | seed는 있었으나 A* 경로 탐색 결과가 2점 미만. ① start/goal이 costmap 범위 밖 ② start 주변이 전부 벽(cost ≥ obstacle_cost)으로 둘러싸임 ③ goal이 없음(`have_goal == false`) | 빈 경로 발행 |
| `WARNING - curvature exceeds r_min` | Stage 6 (Safety Check) | 경로의 최대 곡률(κ_max)이 차량 최소 회전반경의 한계 곡률(κ_limit = 1/r_min)을 초과. 스티어링을 최대로 꺾어도 해당 곡률을 추종하기 어려움 | **경로 발행됨** (추종은 가능하나 주의 필요) |
| `WARNING - too short valid path` | Stage 6 (Safety Check) | A* 경로는 생성되었으나 후처리 결과 총 길이가 `min_path_length`(1.5m) 미만. 장애물이 가까이 밀집하여 짧은 partial path만 생성된 경우 | **경로 발행됨** (짧은 경로임을 알림) |

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
