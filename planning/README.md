# track_planning — 로컬 플래너 패키지 간단 설명서

## 1. 개요

자율주행 EV 경진대회용 **로컬 경로 계획(Local Path Planning)** 패키지.
LiDAR + 카메라로 인식한 차선·콘·장애물 정보를 받아, 실시간으로 주행 경로를 생성한다.

```
센서 인식 결과 ──→ [ track_planning ] ──→ 주행 경로 + 목표 속도
                     (10Hz, 100ms 주기)
```

**대회 환경**: 차로 폭 1.5m, 흰색 테이프 차선, PE 드럼/교통콘 장애물, GPS 사용 금지 구간

---

## 2. 패키지 구조

```
planning/src/
├── track_msgs/                        # 커스텀 ROS 2 메시지 패키지
│   ├── msg/
│   │   ├── Cone.msg                   # 콘 (위치 + 분류)
│   │   ├── ConeArray.msg              # 콘 배열
│   │   ├── LaneBoundary.msg           # 차선 경계 (좌/우, 포인트 배열)
│   │   ├── LaneBoundaryArray.msg      # 차선 경계 배열
│   │   ├── Obstacle.msg               # 장애물 (위치 + 반경)
│   │   ├── ObstacleArray.msg          # 장애물 배열
│   │   ├── PlannerStatus.msg          # 플래너 상태 (모드, 속도, 곡률)
│   │   └── SystemState.msg            # 시스템 FSM 상태
│   ├── CMakeLists.txt
│   └── package.xml
│
└── track_planning/                    # 메인 플래너 패키지
    ├── include/track_planning/
    │   ├── common/                    # 공통 유틸리티
    │   │   ├── types.hpp              #   데이터 구조체 (Point2D, PairResult 등)
    │   │   ├── params.hpp             #   파라미터 로드 (PlanningParams)
    │   │   ├── geometry.hpp           #   2D 기하학 함수 (거리, 각도, 리샘플 등)
    │   │   └── debug_publish.hpp      #   디버그 메시지 변환 헬퍼
    │   ├── corridor/                  # 복도 구축 모듈
    │   │   ├── corridor_builder.hpp   #   Greedy Chaining으로 좌/우 경계 생성
    │   │   ├── pair_validator.hpp     #   좌우 경계 쌍 유효성 검증
    │   │   ├── virtual_boundary.hpp   #   한쪽만 있을 때 가상 경계 생성
    │   │   └── centerline_builder.hpp #   센터라인 생성 (중점 or 오프셋)
    │   ├── costmap/                   # 코스트맵 모듈
    │   │   ├── costmap_validation_builder.hpp  # 5층 코스트맵 빌드
    │   │   ├── inflation.hpp          #   원형 커널 팽창
    │   │   ├── drivable_mask_scanline.hpp     # Scanline Fill 주행 마스크
    │   │   └── connected_component.hpp #  BFS ego 도달 가능 영역 필터
    │   ├── goal/
    │   │   └── goal_selector.hpp      #   Lookahead 기반 목표점 선택
    │   ├── planner/
    │   │   ├── astar_planner.hpp      #   8방향 A* 경로 탐색
    │   │   └── mode_selector.hpp      #   DIRECT vs ASTAR 모드 결정
    │   ├── postprocess/
    │   │   └── path_postprocessor.hpp #   경로 후처리 (가지치기, 스무딩)
    │   ├── safety/
    │   │   └── safety_checker.hpp     #   곡률 기반 속도 제한
    │   └── nodes/
    │       └── local_planner_node.hpp #   메인 노드 (파이프라인 통합)
    ├── src/                           # 구현 파일 (.cpp)
    │   ├── corridor/                  #   corridor_builder, pair_validator,
    │   │                              #   virtual_boundary, centerline_builder
    │   ├── costmap/                   #   costmap_validation_builder, inflation,
    │   │                              #   drivable_mask_scanline, connected_component
    │   ├── goal/                      #   goal_selector
    │   ├── planner/                   #   astar_planner
    │   ├── postprocess/               #   path_postprocessor
    │   └── nodes/                     #   local_planner_node
    ├── config/
    │   └── planning.yaml              # 전체 파라미터 설정 파일
    ├── launch/
    │   └── planning.launch.py         # ComposableNode launch 파일
    ├── CMakeLists.txt
    └── package.xml
```

---

## 3. 파이프라인 (10Hz 타이머 콜백)

`on_timer()` 함수에서 매 100ms마다 아래 단계를 순차 실행한다:

```
┌─────────────────────────────────────────────────────────────┐
│                    on_timer() 파이프라인                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ① Stale 검사                                               │
│     odom/perception 타임아웃 확인 → 초과 시 안전 정지           │
│                          ↓                                  │
│  ② 입력 파싱                                                 │
│     LaneBoundary → 좌/우 분리, Cone → 좌/우 분류              │
│     Obstacle → 원형 포인트 변환, Odom → ego 속도 추출          │
│                          ↓                                  │
│  ③ 복도(Corridor) 구축     [corridor_builder]                │
│     Greedy Chaining으로 차선+콘 점들을 순서대로 연결             │
│     → 좌측 경계 폴리라인 + 우측 경계 폴리라인 생성               │
│                          ↓                                  │
│  ④ 페어 검증               [pair_validator]                  │
│     좌우 경계가 유효한 차선 쌍인지 확인                         │
│     (폭 범위, 각도 차이, 교차 여부)                            │
│                          ↓                                  │
│  ⑤ 가상 경계 생성           [virtual_boundary]                │
│     한쪽만 있으면 EMA 추정 폭으로 반대편 생성                    │
│                          ↓                                  │
│  ⑥ 센터라인 생성            [centerline_builder]              │
│     Case1: 양쪽 있음 → 중점 연결                              │
│     Case2: 한쪽만 → 오프셋으로 센터라인 생성                    │
│                          ↓                                  │
│  ⑦ 코스트맵 생성            [costmap_validation_builder]      │
│     5층: Base → 경계장벽 → 경계팽창 → 장애물 → 장애물팽창       │
│     + Scanline 주행마스크 + BFS ego 연결 필터                  │
│                          ↓                                  │
│  ⑧ 모드 선택               [mode_selector]                   │
│     DIRECT: 센터라인 직접 추종 (장애물 없을 때)                 │
│     ASTAR : A* 경로 탐색 (장애물 회피 필요 시)                 │
│                          ↓                                  │
│  ⑨ 경로 생성                                                 │
│     DIRECT → 센터라인 그대로 사용                              │
│     ASTAR  → 목표점 선택 [goal_selector]                      │
│              → A* 탐색 [astar_planner]                       │
│                          ↓                                  │
│  ⑩ 후처리                  [path_postprocessor]              │
│     Greedy Shortcut 가지치기 → 이동평균 스무딩                 │
│     → 등간격 리샘플 → Yaw 계산                                │
│                          ↓                                  │
│  ⑪ 안전 검사               [safety_checker]                  │
│     Menger 곡률 계산 → 곡률 기반 속도 제한                     │
│     v_safe = √(a_lat_max / κ_max)                           │
│                          ↓                                  │
│  ⑫ 퍼블리시                                                  │
│     /planning/path + /planning/target_speed + debug 토픽들    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. ROS 2 토픽

### 4.1 구독 (Subscriptions) — 5개

| 토픽 이름 | 메시지 타입 | 설명 |
|---|---|---|
| `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray` | 차선 경계 (카메라/LiDAR) |
| `/perception/cones` | `track_msgs/ConeArray` | 콘 위치 (LiDAR) |
| `/perception/obstacles` | `track_msgs/ObstacleArray` | 장애물 위치 (LiDAR) |
| `/odometry` | `nav_msgs/Odometry` | 차량 위치·속도 |
| `/system/state` | `track_msgs/SystemState` | 시스템 FSM 상태 |

### 4.2 발행 (Publishers) — 9개

**핵심 토픽 (3개)**

| 토픽 이름 | 메시지 타입 | 설명 |
|---|---|---|
| `/planning/path` | `nav_msgs/Path` | **최종 주행 경로** (PoseStamped 배열) |
| `/planning/status` | `track_msgs/PlannerStatus` | 플래너 상태 (모드, 속도, 곡률) |
| `/planning/target_speed` | `std_msgs/Float64` | **목표 속도** [m/s] |

**디버그 토픽 (6개)** — RViz2 시각화용

| 토픽 이름 | 메시지 타입 | 설명 |
|---|---|---|
| `/planning/debug/corridor_left` | `nav_msgs/Path` | 좌측 복도 경계 |
| `/planning/debug/corridor_right` | `nav_msgs/Path` | 우측 복도 경계 |
| `/planning/debug/centerline` | `nav_msgs/Path` | 센터라인 |
| `/planning/debug/pair_valid` | `std_msgs/Bool` | 페어 유효 여부 |
| `/planning/debug/virtual_used` | `std_msgs/Bool` | 가상 경계 사용 여부 |
| `/planning/debug/path_mode` | `std_msgs/String` | 현재 모드 ("DIRECT"/"ASTAR") |

### 4.3 QoS & 좌표계

- **QoS**: 모든 토픽 `rclcpp::QoS(1)` — 큐 깊이 1 (Intra-process 최적화)
- **Frame ID**: `"base_link"` (차량 중심 좌표계)

---

## 5. 핵심 알고리즘 요약

### 5.1 Greedy Chaining (복도 구축)
시드 포인트에서 시작하여, 스코어링 함수로 다음 최적 포인트를 탐욕적으로 선택해 나감.
```
score = w_s·(전진) − w_d·(횡편차) − w_a·(각도) − w_p·(예측오차)
```

### 5.2 5층 코스트맵
```
Layer 0: Base (free=0)
Layer 1: 경계 장벽 (Bresenham 래스터화, cost=254)
Layer 2: 경계 팽창 (원형 커널, r = width/2 + margin)
Layer 3: 장애물 (포인트 래스터화, cost=254)
Layer 4: 장애물 팽창 (원형 커널)
+ Scanline 주행마스크 + BFS ego 연결 필터
```

### 5.3 모드 선택 (DIRECT vs ASTAR)
5가지 조건 중 하나라도 해당되면 ASTAR:
1. 센터라인 포인트 < 3개
2. 센터라인 점프 감지 (연속 점 거리 > 1.0m)
3. 코스트맵 충돌 감지
4. 센터라인이 ROI 밖
5. `enable_astar: true` 강제 설정

### 5.4 A* 경로 탐색
- **8방향** 이동 (상하좌우 + 대각선)
- **Octile 휴리스틱**: `h = max(dx,dy) + (√2−1)·min(dx,dy)`
- **셀 비용 가중**: `g += base_cost + cell_cost_weight`
- 목표점은 센터라인 위 Lookahead 거리에서 선택

### 5.5 경로 후처리
```
A* 원시 경로 → Greedy Shortcut 가지치기 → 이동평균 스무딩 → 등간격 리샘플 → Yaw 계산
```

### 5.6 안전 검사 (Menger 곡률)
연속 3점 p1, p2, p3로 곡률 계산:
```
κ = 2·|cross(p2−p1, p3−p2)| / (|p2−p1|·|p3−p2|·|p3−p1|)
v_safe = min(v_max, √(a_lat_max / κ_max))
```

---

## 6. 차량 스펙 (T870)

| 항목 | 값 | 비고 |
|---|---|---|
| 차량 폭 | 0.50 m | 포인트 모델 → inflation 반경에 흡수 |
| 축간 거리 | 0.87 m | Ackermann 조향 모델 |
| 최대 조향각 | 0.314 rad (≈18°) | |
| 최대 속도 | 1.60 m/s | |
| 최대 횡가속도 | 2.0 m/s² | 곡률 기반 속도 제한에 사용 |

---

## 7. 빌드 & 실행

```bash
# 빌드 (ev_ws에서)
cd ~/ev_ws
colcon build --packages-select track_msgs track_planning

# 소싱
source install/setup.bash

# 실행
ros2 launch track_planning planning.launch.py
```

### 빌드 라이브러리 구조
```
track_planning_corridor      ← 복도 구축 (4개 .cpp)
track_planning_costmap       ← 코스트맵 (4개 .cpp)
track_planning_planner       ← 경로 계획 (2개 .cpp)
track_planning_postprocess   ← 후처리 (1개 .cpp)
       ↑ ↑ ↑ ↑
local_planner_component      ← 위 4개를 링크하는 ComposableNode
```

---

## 8. 파라미터 튜닝 가이드

설정 파일: `config/planning.yaml`

| 상황 | 조정할 파라미터 | 방향 |
|---|---|---|
| 경로가 울퉁불퉁 | `postprocess.smooth_window` | ↑ 증가 (5→7) |
| 장애물에 너무 가까움 | `safety.margin` | ↑ 증가 (0.10→0.15) |
| 경로가 너무 보수적 | `safety.margin` | ↓ 감소 |
| 센터라인 불안정 | `corridor.score.w_a` | ↑ 증가 (각도 페널티 강화) |
| A* 경로가 코스트맵 밖 | `roi.x_max`, `roi.y_max` | ↑ 증가 |
| 좁은 구간 통과 불가 | `virtual.min_corridor_width` | ↓ 감소 |
| 속도가 너무 느림 | `speed.a_lat_max` | ↑ 증가 (미끄러짐 주의) |
| ASTAR 모드 빈번 전환 | `mode_selector.centerline_jump_th` | ↑ 증가 |
| 복도가 짧게 끊김 | `corridor.filter.s_max`, `r_search` | ↑ 증가 |

---

## 9. RViz2 디버깅

```bash
# 디버그 토픽으로 파이프라인 상태 확인
ros2 topic echo /planning/debug/path_mode       # 현재 모드
ros2 topic echo /planning/debug/pair_valid       # 페어 유효 여부
ros2 topic echo /planning/debug/virtual_used     # 가상 경계 사용 여부
```

RViz2에서 추가할 Display:
- **Path**: `/planning/path` (최종 경로, 녹색 권장)
- **Path**: `/planning/debug/corridor_left` (좌측 경계, 파란색)
- **Path**: `/planning/debug/corridor_right` (우측 경계, 빨간색)
- **Path**: `/planning/debug/centerline` (센터라인, 노란색)

---
