# chaining_costmap_ver — 코드 구조 및 파이프라인 문서

## 패키지 개요

LiDAR + Camera 기반 자율주행 경로 계획 패키지.
DirectionChainer v3 + Gaussian Costmap + A* 7단계 파이프라인.

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

## 7단계 파이프라인

```
on_timer() — 10Hz (100ms)
│
├── Stage 0: Stale Gate
│     인지 데이터 타임아웃 검사. 오래된 데이터 → "STALE" 발행 후 중단.
│
├── Stage 1: Input Parse  [nodes/input_parser.hpp]
│     BBox + LaneBoundary → ChainPoint 벡터 (sensor_tf 보정)
│
├── Stage 2: DirectionChainer  [chainer/*.cpp]
│     find_seed → build_graph → extract_backbone(L/R) → resample_component(L/R)
│     * backbone chaining 게이트:
│       G1 (거리 게이트):       d(i,j) ≤ d_max
│       G2 (전방 cone 게이트):  angle(v, u_ij) ≤ forward_cone_deg/2
│       G3 (횡오차 게이트):     |lateral_proj| ≤ lateral_gate
│       G4 (시드 기준 횡편차 가드): candidate.y が seed_y ± max_lateral_deviation 이내
│          left  backbone → candidate.y < seed_y - max_lateral_deviation 이면 reject
│          right backbone → candidate.y > seed_y + max_lateral_deviation 이면 reject
│          → backbone이 반대편으로 크로스하는 것을 사전 차단
│     * backbone chaining 시 2-phase BBOX 최우선 탐색:
│       Phase 1 — d_max 범위 내 모든 bbox를 knn 없이 직접 전수 탐색
│                 (lane point가 많아도 bbox가 k개 제한에 밀리지 않음)
│                 G1+G2+G3+G4 게이트 적용
│       Phase 2 — bbox 후보 없으면 knn fallback → bbox-first 선택
│                 G1+G2+G3+G4 게이트 적용
│     * 교차 판정 (1·2단계 backbone 확정 직후):
│       owner[right_seed] == LEFT_BACKBONE → left_crossed_right = true
│       owner[left_seed]  == RIGHT_BACKBONE → right_crossed_left = true
│       한쪽 backbone이 반대쪽 seed까지 체이닝한 비정상 상황을 감지.
│       플래그는 DirectionChainResult에 저장 → Stage 3c에서 사용.
│
├── Stage 2.5: Seed Gate
│     양쪽 backbone 실패 시 "FAIL" 발행 후 중단.
│
├── Stage 3: Costmap + A*  [costmap/, planner/, nodes/goal_calculator.hpp]
│     3a. ChainPoint → ChainedPoint 변환 (is_backbone 플래그 전파)
│     3b. Gaussian Costmap 생성 + entry walls
│         * backbone 포인트(is_backbone=true)는 타입(LANE/BBOX)에 관계없이
│           bbox_cost_max + bbox_radius 적용 → 전환 구간 gap 방지
│     3c. Goal 계산
│         * 교차 판정 처리: left_crossed_right 또는 right_crossed_left가
│           true이면 해당 backbone 인덱스 중간점을 local_goal로 즉시 반환
│         * 정상 시: 좌/우 끝점 선분 중점 or 폴백
│     3d. Goal → costmap 경계 clamp
│     3e. A* 경로 탐색
│
├── Stage 5: Postprocess  [postprocess/*.cpp]
│     prune → resample → smooth → curvature_clamp → yaw
│
├── Stage 6: Safety Check  [safety/safety_checker.hpp]
│     경로 길이 + Menger 곡률 검사 → OK / FAIL / WARNING
│
└── Stage 7: Publish  [nodes/debug_publisher.hpp]
      Core: path (Marker), status (String)
      Debug: costmap, obstacle_wall, curvature, raw_path,
             pruned_path, chains, seeds, local_goal
```

## 빌드 타겟

| 타겟 | 타입 | 소스 파일 |
|------|------|-----------|
| `chaining_costmap_chainer` | 공유 라이브러리 | `src/chainer/*.cpp` (5파일) |
| `chaining_costmap_costmap` | 공유 라이브러리 | `src/costmap/costmap_generator.cpp` |
| `chaining_costmap_astar` | 공유 라이브러리 | `src/planner/astar_planner.cpp` |
| `chaining_costmap_postprocess` | 공유 라이브러리 | `src/postprocess/*.cpp` (4파일) |
| `lc_planner_component` | ComposableNode | `src/nodes/*.cpp` (4파일) |

## 빌드 명령

```bash
cd ~/ev_ws/planning && colcon build --symlink-install --packages-select chaining_costmap_ver
```

## 모듈 분리 원칙

- **하나의 클래스, 여러 .cpp 파일**: DirectionChainer와 PathPostprocessor는 같은 헤더를 공유하면서 구현을 역할별로 분리. API 변경 없이 디버깅 편의성 향상.
- **일관된 .hpp/.cpp 분리**: input_parser, goal_calculator, debug_publisher도 선언(.hpp)과 구현(.cpp)을 분리. `lc_planner_component`의 소스 목록에 포함.
- **우산 헤더**: types.hpp는 4개 서브 헤더를 포함하는 우산 헤더. 기존 코드의 `#include "types.hpp"` 호환성 유지.
