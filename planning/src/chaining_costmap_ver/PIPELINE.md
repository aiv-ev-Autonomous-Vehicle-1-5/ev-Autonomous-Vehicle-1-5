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
│     BBox/LaneBoundary → ChainPoint 변환
│     LaneBoundary.msg의 lane_side 필드에서 LEFT/RIGHT 라벨을 직접 읽어 ChainPoint.lane_side에 설정
│     bbox는 velodyne 프레임 → on_timer()에서 tf2_ros::Buffer::lookupTransform("base_link","velodyne")으로 변환
│
├── Stage 2: DirectionChainer v4  [chainer/*.cpp]
│     독립 체이닝 + Backtracking 중복 해소 + L/R 백본 추출
│     find_seed (단일 패스, bbox/lane 동일 가중치)
│       → extract_backbone(L, 독립) → extract_backbone(R, 독립)
│       → resolve_overlaps (backtracking) → trim_crossing → resample
│     * find_seed 전략 (단일 패스, bbox/lane 동일 가중치):
│       bbox와 lane을 동일하게 취급하여 조건 내 가장 가까운 점 선택
│       (x ≥ seed_rear_limit, side_seed_y 가드, d ≤ seed_max_dist, 반대편 lane 제외)
│     * backbone chaining 게이트:
│       G1 (거리 게이트):       d(i,j) ≤ d_max_bbox or d_max_lane (현재 노드 타입 기준)
│       G2 (전방 cone 게이트):  angle(v, u_ij) ≤ forward_cone_deg/2
│       G3 (횡오차 게이트):     |lateral_proj| ≤ lateral_gate
│       G4 (시드 기준 횡편차 가드): candidate.y ∈ seed_y ± max_lateral_deviation
│     * backbone chaining 단일 패스 탐색 + 클러스터 락:
│       d_max 범위 내 모든 후보(bbox+lane)를 동일 가중치로 탐색
│       G0(lane_side)+G1(거리)+G2(cone)+G3(lateral) 게이트 적용 후 최소 비용 선택
│       클러스터 락: best가 자기 쪽 lane point이면 해당 클러스터(label)만
│       후보로 제한하여 기존 게이트로 클러스터 전체 chaining → 소진 시 락 해제
│     * 독립 체이닝 (v4):
│       좌/우 각각 독립 owner 배열로 체이닝 → 상대 chain의 영향 없음
│     * 2.5단계 — Backtracking 중복 해소 (resolve_overlaps):
│       독립 체이닝 후 중복 노드 탐지 (left_bb ∩ right_bb)
│       3-node 윈도우(A→B→C) 곡률 변화 + 거리 비용 비교
│       비용 높은 쪽: truncate + 중복 노드 제외 후 재체이닝
│       비용 동일: 양쪽 모두 truncate (재체이닝 없음)
│       max_backtrack_count까지 반복
│     * 교차 판정 (2.6단계, backtracking 후):
│       left_bb에 right_seed 포함 여부 → left_crossed_right
│       right_bb에 left_seed 포함 여부 → right_crossed_left
│     * 선분 교차 검증 (2.75단계, resample 전):
│       CCW 기반으로 좌/우 backbone의 모든 선분 쌍을 교차 검사
│       교차 발견 시 양쪽 모두 가장 이른 교차 지점에서 tail trim
│       제거된 노드는 owner → NONE 복원 → unchained로 수집
│
├── Stage 2.5: Seed Gate
│     양쪽 backbone 실패 시 "FAIL" 발행 후 중단
│     시드 유효성 검증
│
├── Stage 3: Costmap + A*  [costmap/, planner/, nodes/goal_calculator.hpp]
│     3a. ChainPoint → ChainedPoint 변환 (is_backbone 플래그 전파)
│     3b. Gaussian Costmap 생성 (apply_source 최적화 적용)
│         * BBOX: bbox_cost_max + bbox_radius
│         * LANE + backbone 연결: bbox_cost_max + lane_radius (max cost, radius 독립)
│         * LANE + backbone 미연결: lane_cost_max + lane_radius (기존 약한 비용)
│         * apply_source 성능 최적화:
│           - inv_resolution 미리 계산하여 루프 내 나눗셈 제거
│           - P3: flat zone 판정에서 sqrt 제거 — inner_radius_sq로 d² 비교
│                 sqrt는 가우시안 감쇠 구간(d > inner_radius)에서만 호출
│           - P4: row-level 원형 col 범위 클리핑
│                 사각형 bounding box 대신, 각 row에서 dx_max = sqrt(r_total_sq - dy²)로
│                 유효 col 범위를 좁혀 순회 셀 수를 줄임
│                 기존 내부 루프의 원형 클리핑(if d2 > r_total_sq continue) 불필요
│     3b-2. Entry walls
│     3b-3. 중앙선 유인 비용 (center line attraction) ← 가장 마지막에 적용
│         * 양쪽 backbone 존재 시 2-Phase 중앙선 생성:
            - Phase 1: 짧은 chain 길이(min_len)까지 양쪽 midpoint 연결 (chain 간 거리 > center_gap_threshold 시 스킵)
            - Phase 2: 긴 chain의 나머지 구간을 안쪽으로 track_half_width 오프셋
            음의 가우시안 비용 적용
│         * 한쪽 backbone만 존재 시: track_half_width(0.75m) 수직 오프셋으로 centerline 계산
│           - 각 backbone 점의 접선(tangent) 방향을 구한 뒤 90° 회전하여 법선 산출
│           - left only → 시계 방향 90° (트랙 안쪽=우측), right only → 반시계 방향 90° (트랙 안쪽=좌측)
│         * 다른 비용이 덮어쓰지 못하도록 최종 단계에서 차감
│     3c. Goal 계산
│         * 교차 판정 처리: 교차 시 해당 backbone 누적거리 중간점을 local_goal로 반환
│         * 정상 시: centerline 마지막 점을 goal로 사용 (양쪽/한쪽 backbone 공통)
│     3d. Goal → costmap 경계 clamp
│     3e. A* 경로 탐색
│
├── Stage 5: Postprocess  [postprocess/*.cpp]  (costmap-aware)
│     prune → resample → smooth → curvature_clamp → yaw
│     * costmap-aware 후처리: costmap + obstacle_cost를 참조하여 장애물 침범 방지
│       - prune: shortcut 직선을 resolution 간격으로 샘플링, obstacle 셀 통과 시 shortcut 거부
│       - smooth: 이동평균 결과가 obstacle 셀이면 원래 좌표 유지
│       - curvature_clamp: 중점 방향 이동 결과가 obstacle 셀이면 이동 거부
│
├── Stage 6: Safety Check  [safety/safety_checker.hpp]
│     경로 유효성 + 곡률 검증 → OK / FAIL / WARNING
│
└── Stage 7: Publish  [nodes/debug_publisher.hpp]
      Core: path (Marker), status (String)
      Debug: costmap, obstacle_wall, curvature, raw_path,
             pruned_path, chains, seeds, local_goal, lane_points, pipeline_timing
```

---

## 성능 디버깅

### 1) 실시간 토픽

`/planning/debug/pipeline_timing` (std_msgs/String, lazy — 구독자가 있을 때만 발행)

```bash
ros2 topic echo /planning/debug/pipeline_timing --field data
```

### 2) 파일 로깅 (항상 기록)

노드 시작 시 `~/dbg_logs/timing_YYYYMMDD_HHMMSS.log` 파일을 자동 생성.
토픽 구독 여부와 무관하게 매 사이클(10Hz) 기록, 10사이클(1초)마다 flush.

```bash
# 최신 로그 확인
tail -f ~/dbg_logs/timing_*.log

# chain 실패한 사이클만 필터
grep "valid:0" ~/dbg_logs/timing_*.log
```

### 출력 포맷 (한 줄)

```
[Timing] total=15.32ms | input=0.12 chainer=3.45 (seed=0.01 L_bb=0.95 R_bb=0.88 overlap=0.52 trim=0.01 L_resamp=0.12 R_resamp=0.14) costmap=5.21 entry=0.34 center=1.82 goal=0.05 astar=2.88 post=0.92 safety=0.01 | pts=45 seeds(L:3 R:7) bb(L:12 R:15) comp(L:120 R:150) unch:18 center:95 astar:87 valid:1 OK
```

| 필드 | 설명 |
|------|------|
| `stage1_input` | Input Parse + TF 변환 |
| `stage2_chainer` | DirectionChainer 전체 (내부 서브스텝 괄호 안) |
| `seed` | find_seed(L) + find_seed(R) |
| `L_bb` / `R_bb` | extract_backbone (좌/우) |
| `overlap` | resolve_overlaps() backtracking |
| `trim` | trim_crossing_backbones() 선분 교차 검증 |
| `L_resamp` / `R_resamp` | resample_component (좌/우) |
| `stage3_costmap` | Gaussian costmap generate() (P3/P4 최적화 적용: sqrt 최소화 + 원형 col 클리핑) |
| `stage3_entry` | Entry walls 생성 |
| `stage3_center` | 중앙선 유인 비용 적용 |
| `stage3_goal` | Goal 계산 + clamp |
| `stage3_astar` | A* 경로 탐색 |
| `stage5_post` | Postprocess (prune→smooth→curvature_clamp→resample) |
| `stage6_safety` | Safety check |
| `pts` | 입력 포인트 수 |
| `seeds` | 좌/우 seed 인덱스 (-1=미검출) |
| `bb` | 좌/우 backbone 노드 수 |
| `comp` | 좌/우 리샘플 컴포넌트 수 |
| `unchained` | unchained 포인트 수 (costmap 장애물) |

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
│   │   │   └── chain_types.hpp       # StopReason, NodeOwner, SideResult, DirectionChainResult
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
│   ├── chainer/                      # DirectionChainer 구현 (4파일, 1클래스)
│   │   ├── direction_chainer.cpp     # chain() 오케스트레이터
│   │   ├── seed_selector.cpp         # find_seed(), knn()
│   │   ├── backbone_extractor.cpp    # extract_backbone(), chain_one_direction(),
│   │   │                             #   compute_cost(), compute_cost_prime(),
│   │   │                             #   resolve_overlaps(), compute_backtrack_cost(),
│   │   │                             #   rechain_from()
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
| `costmap/` | Gaussian Costmap 생성 (apply_source: P3 sqrt 제거 + P4 원형 col 클리핑 최적화) |
| `planner/` | A* 경로 탐색 |
| `postprocess/` | 경로 후처리 (prune, smooth, curvature_clamp, resample, yaw) — costmap-aware: obstacle 셀 침범 방지 |
| `safety/` | 경로 유효성 + 곡률 검증 |
| `nodes/` | 노드 오케스트레이터 + 입력 파서 + 골 계산 + 디버그 발행 |

---

## 빌드 타겟

| 타겟 | 타입 | 소스 파일 |
|------|------|-----------|
| `chaining_costmap_chainer` | 공유 라이브러리 | `src/chainer/*.cpp` (4파일) |
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

### 좌표 변환 (TF2)

LiDAR bbox 좌표 변환은 `tf2_ros::Buffer::lookupTransform("base_link", "velodyne")`을 사용한다.
`LCPlannerNode`의 `on_timer()` 콜백에서 raw bbox 및 tracked bbox 모두에 동일한 TF를 적용.
기존 `sensor_tf` 파라미터(tf_x, tf_y, tf_z)는 더 이상 사용하지 않으며, TF tree에서 자동으로 가져온다.

### chainer

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `side_seed_y` | 0.1 | m | 시드 Y 오프셋 |
| `seed_rear_limit` | -2.0 | m | seed 후보 후방 제한 (x ≥ 이 값인 점만 후보) |
| `seed_max_dist` | 1.5 | m | seed 탐색 최대 거리 (Pass 1/2 공통) |
| `k` | 10 | - | KNN 이웃 수 |
| `d_max_bbox` | 2.5 | m | BBOX 노드에서의 탐색 최대 거리 |
| `d_max_lane` | 0.5 | m | LANE 노드에서의 탐색 최대 거리 |
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
| `track_half_width` | 0.75 | m | 트랙 반폭 — 한쪽 chain만으로 centerline 계산 시 수직 오프셋 |
| `center_gap_threshold` | 2.5 | m | 양쪽 chain 간 거리 초과 시 centerline midpoint 생성 안 함 (노이즈 제거) |

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
