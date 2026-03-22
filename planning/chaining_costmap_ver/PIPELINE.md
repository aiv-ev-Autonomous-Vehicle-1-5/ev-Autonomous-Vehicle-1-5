# chaining_costmap_ver — 코드 구조 및 파이프라인 문서

## 패키지 개요

- **Package**: chaining_costmap_ver
- **Description**: DirectionChainer v3 + Gaussian Costmap + A* 기반 로컬 경로 계획 패키지
- LiDAR + Camera 인식 결과를 사용한 7단계 파이프라인
- ROS 2 Humble, C++17, Component architecture
- **Node**: LCPlannerNode (ComposableNode, 10Hz)

---

## 7단계 파이프라인

```
on_timer() — 10Hz (100ms)
│
├── Stage 0: Stale Gate
│     인식 데이터 유효시간 검사 (perception_ms: 300ms)
│     오래된 데이터 → "STALE" 발행 후 중단
│
├── Stage 1: Input Parse  [nodes/input_parser.hpp]
│     BBox/LaneBoundary → ChainPoint 변환 (sensor_tf 보정)
│
├── Stage 2: DirectionChainer  [chainer/*.cpp]
│     컴포넌트 분류 + L/R 백본 추출
│     find_seed (2-pass bbox 우선) → build_graph → extract_backbone(L/R) → resample_component(L/R)
│     * find_seed 전략 (2-pass bbox 우선):
│       Pass 1 — bbox만 탐색 (x ≥ -2.0, side_seed_y 가드, d ≤ seed_bbox_max_dist)
│                조건 만족 bbox 중 가장 가까운 것 반환
│       Pass 2 — Pass 1 실패 시 bbox+lane 전체에서 가장 가까운 점 (기존 로직)
│     * backbone chaining 게이트:
│       G1 (거리 게이트):       d(i,j) ≤ d_max
│       G2 (전방 cone 게이트):  angle(v, u_ij) ≤ forward_cone_deg/2
│       G3 (횡오차 게이트):     |lateral_proj| ≤ lateral_gate
│       G4 (시드 기준 횡편차 가드): candidate.y ∈ seed_y ± max_lateral_deviation
│     * backbone chaining 시 2-phase BBOX 최우선 탐색:
│       Phase 1 — d_max 범위 내 모든 bbox를 직접 전수 탐색
│       Phase 2 — bbox 후보 없으면 knn fallback → bbox-first 선택
│     * 교차 판정 (backbone 확정 직후):
│       한쪽 backbone이 반대쪽 seed까지 체이닝한 비정상 상황을 감지
│
├── Stage 2.5: Seed Gate
│     양쪽 backbone 실패 시 "FAIL" 발행 후 중단
│     시드 유효성 검증
│
├── Stage 3: Costmap + A*  [costmap/, planner/, nodes/goal_calculator.hpp]
│     3a. ChainPoint → ChainedPoint 변환 (is_backbone 플래그 전파)
│     3b. Gaussian Costmap 생성
│         * backbone 포인트는 bbox_cost_max + bbox_radius 적용
│     3b-2. 중앙선 유인 비용 (center line attraction)
│         * 좌/우 backbone 중점 연결선에 음의 가우시안 비용 적용
│     3b-3. Entry walls
│     3c. Goal 계산
│         * 교차 판정 처리: 교차 시 해당 backbone 중간점을 local_goal로 반환
│         * 정상 시: 좌/우 끝점 선분 중점 or 폴백
│     3d. Goal → costmap 경계 clamp
│     3e. A* 경로 탐색
│
├── Stage 5: Postprocess  [postprocess/*.cpp]
│     prune → smooth → curvature_clamp → resample → yaw
│
├── Stage 6: Safety Check  [safety/safety_checker.hpp]
│     경로 유효성 + 곡률 검증 → OK / FAIL / WARNING
│
└── Stage 7: Publish  [nodes/debug_publisher.hpp]
      Core: path (Marker), status (String)
      Debug: costmap, obstacle_wall, curvature, raw_path,
             pruned_path, chains, seeds, local_goal
```

---

## 디렉토리 구조

```
chaining_costmap_ver/
├── include/chaining_costmap_ver/
│   ├── common/
│   │   ├── types.hpp                 # 우산 헤더 (아래 4개 포함)
│   │   ├── types/
│   │   │   ├── point_types.hpp       # Point2D, PointType, ChainedPoint, ChainPoint
│   │   │   ├── costmap_types.hpp     # CostmapResult
│   │   │   ├── planner_types.hpp     # PostprocessResult, PlannerState
│   │   │   └── chain_types.hpp       # ChainingGraph, StopReason, NodeOwner, SideResult, DirectionChainResult
│   │   ├── params.hpp                # PlanningParams (모든 파라미터 구조체)
│   │   ├── geometry.hpp              # 2D 기하 유틸리티 (inline)
│   │   └── debug_publish.hpp         # to_path_msg(), to_points_marker()
│   ├── chainer/
│   │   └── direction_chainer.hpp     # DirectionChainer 클래스 선언
│   ├── costmap/
│   │   └── costmap_generator.hpp     # CostmapGenerator 클래스 선언
│   ├── planner/
│   │   └── astar_planner.hpp         # AStarPlanner 클래스 선언
│   ├── postprocess/
│   │   └── path_postprocessor.hpp    # PathPostprocessor 클래스 선언
│   ├── safety/
│   │   └── safety_checker.hpp        # safety_checker::check() (inline)
│   └── nodes/
│       ├── chaining_costmap_ver_node.hpp  # LCPlannerNode 클래스 선언
│       ├── input_parser.hpp          # Stage 1: parse_input() 선언
│       ├── goal_calculator.hpp       # Stage 3c: calculate_goal() 선언
│       └── debug_publisher.hpp       # Stage 7: 디버그 시각화 헬퍼 선언
├── src/
│   ├── chainer/                      # DirectionChainer 구현 (5파일, 1클래스)
│   │   ├── direction_chainer.cpp     # chain() 오케스트레이터
│   │   ├── seed_selector.cpp         # find_seed(), knn()
│   │   ├── graph_builder.cpp         # build_graph()
│   │   ├── backbone_extractor.cpp    # extract_backbone(), chain_one_direction(),
│   │   │                             #   compute_cost(), compute_cost_prime()
│   │   └── chain_resampler.cpp       # resample_component()
│   ├── costmap/
│   │   └── costmap_generator.cpp     # CostmapGenerator 구현
│   ├── planner/
│   │   └── astar_planner.cpp         # AStarPlanner 구현
│   ├── postprocess/                  # PathPostprocessor 구현 (4파일, 1클래스)
│   │   ├── path_postprocessor.cpp    # process() 오케스트레이터
│   │   ├── prune.cpp                 # ① Douglas-Peucker 유사 단순화
│   │   ├── smooth.cpp                # ③ 이동 평균 필터
│   │   └── curvature_clamp.cpp       # ④ 최대 곡률 제한
│   └── nodes/                        # LCPlannerNode 구현 (4파일)
│       ├── chaining_costmap_ver_node.cpp  # 생성자 + on_timer 파이프라인
│       ├── input_parser.cpp          # Stage 1: parse_input()
│       ├── goal_calculator.cpp       # Stage 3c: calculate_goal(), clamp_goal_to_costmap()
│       └── debug_publisher.cpp       # Stage 7: 디버그 마커 생성/발행
├── config/                           # yaml 파라미터 파일
├── launch/                           # ROS 2 launch 파일
└── CMakeLists.txt                    # 빌드 설정
```

### 모듈 구조

| 모듈 | 역할 |
|------|------|
| `chainer/` | DirectionChainer v3 — 컴포넌트 분류 + L/R 백본 추출 |
| `costmap/` | Gaussian Costmap 생성 |
| `planner/` | A* 경로 탐색 |
| `postprocess/` | 경로 후처리 (prune, smooth, curvature_clamp, resample, yaw) |
| `safety/` | 경로 유효성 + 곡률 검증 |
| `nodes/` | 노드 오케스트레이터 + 입력 파서 + 골 계산 + 디버그 발행 |

---

## 빌드 타겟

| 타겟 | 타입 | 소스 파일 |
|------|------|-----------|
| `chaining_costmap_chainer` | 공유 라이브러리 | `src/chainer/*.cpp` (5파일) |
| `chaining_costmap_costmap` | 공유 라이브러리 | `src/costmap/costmap_generator.cpp` |
| `chaining_costmap_astar` | 공유 라이브러리 | `src/planner/astar_planner.cpp` |
| `chaining_costmap_postprocess` | 공유 라이브러리 | `src/postprocess/*.cpp` (4파일) |
| `lc_planner_component` | ComposableNode | `src/nodes/*.cpp` (4파일) |

### 빌드 명령

```bash
cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select chaining_costmap_ver
```

---

## 주요 파라미터

설정 파일: `config/chaining_costmap_ver.yaml`

### sensor_tf

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `tf_x` | 0.0 | m | 센서 X 오프셋 |
| `tf_y` | 0.0 | m | 센서 Y 오프셋 |
| `tf_z` | 0.7 | m | 센서 Z 오프셋 |

### chainer

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `side_seed_y` | 0.1 | m | 시드 Y 오프셋 |
| `seed_bbox_max_dist` | 3.0 | m | seed bbox 우선 탐색 최대 거리 (Pass 1) |
| `k` | 10 | - | KNN 이웃 수 |
| `d_max` | 2.0 | m | 최대 연결 거리 |
| `forward_cone_deg` | 130 | deg | 전방 cone 각도 |
| `lateral_gate` | 1.2 | m | 횡오차 게이트 |

### costmap

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `size_x` | 16.0 | m | 코스트맵 X 크기 |
| `size_y` | 16.0 | m | 코스트맵 Y 크기 |
| `resolution` | 0.15 | m | 셀 해상도 |
| `bbox_cost_max` | 100 | - | 장애물 최대 비용 |
| `lane_cost_max` | 70 | - | 차선 최대 비용 |

### astar

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `max_iterations` | 10000 | - | 최대 반복 횟수 |
| `goal_tolerance` | 0.3 | m | 골 허용 오차 |

### postprocess

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `resample_ds` | 0.10 | m | 리샘플링 간격 |
| `smooth_window` | 5 | - | 이동 평균 윈도우 크기 |
| `prune_max_dev` | 0.15 | m | 프루닝 최대 편차 |

### safety

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `min_path_length` | 1.5 | m | 최소 경로 길이 |

### vehicle

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `width` | 0.79 | m | 차량 폭 |
| `wheelbase` | 0.73 | m | 축간거리 |
| `delta_max` | 0.3249 | rad | 최대 조향각 |

---

## 모듈 분리 원칙

- **하나의 클래스, 여러 .cpp 파일**: DirectionChainer와 PathPostprocessor는 같은 헤더를 공유하면서 구현을 역할별로 분리. API 변경 없이 디버깅 편의성 향상.
- **일관된 .hpp/.cpp 분리**: input_parser, goal_calculator, debug_publisher도 선언(.hpp)과 구현(.cpp)을 분리. `lc_planner_component`의 소스 목록에 포함.
- **우산 헤더**: types.hpp는 4개 서브 헤더를 포함하는 우산 헤더. 기존 코드의 `#include "types.hpp"` 호환성 유지.
