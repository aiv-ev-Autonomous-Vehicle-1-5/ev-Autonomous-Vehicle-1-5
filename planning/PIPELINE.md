# Planning Pipeline — chaining_costmap_ver

## 전체 흐름도

```
/perception/lane_boundaries        /perception/bboxes
  (LaneBoundaryArray)                (BBoxArray)
          │                              │
          └──────────┬───────────────────┘
                     ▼
        ┌────────────────────────┐
        │  LCPlannerNode (10Hz)  │  ← lc_planner_node
        │  Frame: base_link      │
        └────────────────────────┘
                     │
    ┌────────────────┼────────────────────────────────┐
    │                │                                │
    ▼                ▼                                ▼
 Stage 0          Stage 1                     Debug Topics
 Stale Gate       Input Parse                  (lazy publish)
    │                │
    ▼                ▼
 Stage 2: DirectionChainer  ──→  left/right backbone + branches
    │
    ▼
 Stage 3a: CostmapGenerator  ──→  2D Gaussian cost grid
    │
    ▼
 Stage 3b: AStarPlanner  ──→  raw grid path
    │
    ▼
 Stage 5: PathPostprocessor  ──→  smooth + resample + yaw
    │
    ▼
 Stage 6: SafetyChecker  ──→  curvature/speed 검증
    │
    ▼
 Stage 7: Publish
    │
    ├──→  /planning/path    (nav_msgs/Path)
    └──→  /planning/status  (std_msgs/String)
```

---

## Topics

### Subscriptions (Inputs)

| Topic | Message Type | QoS | 설명 |
|-------|-------------|-----|------|
| `/perception/lane_boundaries` | `ev_msgs::msg::LaneBoundaryArray` | BestEffort, depth=1 | 카메라 차선 경계점 |
| `/perception/bboxes` | `ev_msgs::msg::BBoxArray` | BestEffort, depth=1 | LiDAR 장애물 (콘/드럼) |

### Publications (Outputs) — Core

| Topic | Message Type | 설명 |
|-------|-------------|------|
| `/planning/path` | `nav_msgs::msg::Path` | 최종 후처리된 경로 (controller 입력) |
| `/planning/status` | `std_msgs::msg::String` | 플래너 상태: `"ok"`, `"STALE"`, `"INFEASIBLE"` 등 |

### Publications (Outputs) — Debug (구독자 있을 때만 발행)

| Topic | Message Type | 설명 |
|-------|-------------|------|
| `/planning/debug/costmap` | `nav_msgs::msg::OccupancyGrid` | 2D Gaussian 코스트맵 (0~100) |
| `/planning/debug/raw_path` | `nav_msgs::msg::Path` | A* 출력 (후처리 전 raw 경로) |
| `/planning/debug/left_chain` | `nav_msgs::msg::Path` | 왼쪽 backbone 체인 |
| `/planning/debug/right_chain` | `nav_msgs::msg::Path` | 오른쪽 backbone 체인 |
| `/chaining/debug/left_branches` | `visualization_msgs::msg::MarkerArray` | 왼쪽 branch 시각화 (연두색 LINE_STRIP) |
| `/chaining/debug/right_branches` | `visualization_msgs::msg::MarkerArray` | 오른쪽 branch 시각화 (분홍색) |
| `/chaining/debug/seeds` | `visualization_msgs::msg::MarkerArray` | 체이닝 시드 (초록/빨강 SPHERE) + 골 (파랑) |
| `/planning/debug/local_goal` | `visualization_msgs::msg::MarkerArray` | A* 목표점 (노랑 SPHERE) |

---

## 8-Stage Pipeline 상세

### Stage 0: Stale Gate
- perception_ms (기본 300ms) 이내에 데이터 수신 여부 확인
- 타임아웃 시 `"STALE"` 상태 발행 후 리턴

### Stage 1: Input Parse
- BBoxArray → ChainPoint[] 변환 (sensor_tf 오프셋 적용 → base_link 좌표계)
- LaneBoundaryArray → ChainPoint[] 변환 (이미 base_link 기준)

### Stage 2: DirectionChainer (7-step)

| Step | 이름 | 설명 |
|------|------|------|
| 0 | Preprocess | confidence < min_confidence인 포인트 제거 |
| 1 | Seed Selection | 좌/우 각각 ego에서 가장 가까운 전방 포인트 선택 (\|y\| > side_seed_y) |
| 2 | Graph Building | k-NN + 거리(G1) + 측면(G3) 게이트로 무방향 그래프 생성 |
| 3 | Component Extraction | 각 seed에서 BFS로 연결 컴포넌트 추출 |
| 4 | Backbone Extraction | Greedy 체이닝: 비용 함수 w'(i,j) 최소화 |
| 5 | Branch Extraction | 미체이닝 노드 → 가장 가까운 backbone 노드로 BFS 연결 |
| 6 | Resampling | resample_ds 간격으로 선형 보간 |

**비용 함수:**
```
w'(i,j) = α·C_d + β·C_a + γ·C_lat + δ·C_size + λ_side·C_side

C_d    = d(i,j) / d_max          (거리)
C_a    = angle_error / θ_max     (방향 오차)
C_lat  = |lat_proj| / lateral_gate (측면 편차)
C_size = cone 크기 변화           (콘 전용)
C_side = 중앙선 교차 패널티       (좌우 비대칭)
```

### Stage 3a: CostmapGenerator
- **콘 포인트**: flat zone (cone_radius) + Gaussian 감쇠
  ```
  d ≤ cone_radius  →  cost = cone_cost_max
  d > cone_radius  →  cost = cone_cost_max · exp(-d_eff² / (2σ²))
  ```
- **차선 포인트**: Gaussian 감쇠만 (flat zone 없음)
  ```
  cost = lane_cost_max · exp(-d² / (2σ²))
  ```
- **Unchained 포인트**: 콘으로 취급 (보수적 처리)
- **Entry walls**: seed → ego 방향 가상 벽 (A* 경로를 안쪽으로 유도)

### Stage 3b: AStarPlanner
- **8방향 그리드 탐색** (상하좌우 + 대각선)
- **시작점**: (0, 0) — 차량 위치
- **목표점**: 좌/우 backbone 끝점의 중점
- **f(n) = g(n) + h(n)**
  - g(n): 누적 비용 = g(parent) + 이동비용 + costmap_cost × cost_weight
  - h(n): 유클리드 거리 (admissible heuristic)
- **장애물**: cost ≥ obstacle_cost (기본 80) → 통과 불가
- **종료**: goal_tolerance (0.3m) 이내 도달 또는 max_iterations 초과

### Stage 5: PathPostprocessor (4-step)

| Step | 이름 | 알고리즘 | 설명 |
|------|------|---------|------|
| 1 | Prune | Greedy shortcutting | 직선 구간의 불필요한 점 제거 |
| 2 | Smooth | Moving average (window=5) | 그리드 지그재그 아티팩트 제거 |
| 2.5 | Curvature Clamp | 원 중심 방향으로 중점 이동 | κ > κ_max인 구간 완화 |
| 3 | Resample | 선형 보간 (ds=0.10m) | 균일 간격 waypoint 생성 |
| 4 | Yaw Calc | atan2(dy, dx) | 각 waypoint의 heading 각도 |

### Stage 6: SafetyChecker
- **Menger 곡률**: κ = 2|cross(BA, CB)| / (|AB|·|BC|·|AC|)
- **최소 회전 반경**: r_min = wheelbase / tan(δ_max) ≈ 2.68m
- **곡률 한계**: κ_limit = 1/r_min ≈ 0.373 rad/m
- **속도 제한**: v_safe = min(v_max, √(a_lat_max / κ_max))
- **결과**: OK / STOP / INFEASIBLE

### Stage 7: Publish
- Core 토픽 항상 발행
- Debug 토픽은 구독자 존재 시에만 (lazy publishing)

---

## Parameters (chaining_costmap_ver.yaml)

### costmap
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| size_x | 10.0 | m | 전후 범위 |
| size_y | 10.0 | m | 좌우 범위 |
| resolution | 0.15 | m/cell | 셀 크기 (차로 1.5m = ~10셀) |
| cone_cost_max | 100.0 | — | 콘 중심 코스트 |
| lane_cost_max | 50.0 | — | 차선 경계 코스트 |
| cone_radius | 1.025 | m | 콘 flat zone 반경 |
| sigma | 1.0 | m | Gaussian 표준편차 |
| cost_threshold | 2.0 | — | 코스트 하한 (이하 = 0) |
| entry_wall_ego_y | 1.3 | m | entry wall 측면 오프셋 |

### astar
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| max_iterations | 10000 | — | 최대 반복 횟수 |
| goal_tolerance | 0.3 | m | 목표 도달 허용치 |
| cost_weight | 0.05 | — | 코스트맵 비용 가중치 |
| obstacle_cost | 100.0 | — | 장애물 판정 임계값 |

### vehicle
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| width | 0.75 | m | T870 차폭 |
| wheelbase | 0.87 | m | 축거 |
| delta_max | 0.314 | rad | 최대 조향각 (~18°) |

### safety / speed
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| margin | 0.10 | m | 안전 마진 |
| v_max | 1.60 | m/s | 최대 속도 (~5.76 km/h) |
| a_lat_max | 2.0 | m/s² | 최대 횡가속도 |

### postprocess
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| resample_ds | 0.10 | m | 리샘플 간격 |
| smooth_window | 5 | — | 이동평균 윈도우 크기 |
| prune_max_dev | 0.15 | m | 프루닝 최대 편차 |

### sensor_tf
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| tf_x | 0.0 | m | LiDAR → base_link 전방 오프셋 |
| tf_y | 0.0 | m | LiDAR → base_link 측면 오프셋 |
| tf_z | 0.7 | m | LiDAR 높이 |

### chainer
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| side_seed_y | 0.3 | m | 시드 선택 \|y\| 최소값 |
| k | 8 | — | k-NN 후보 수 |
| d_max | 2.0 | m | 최대 이웃 거리 |
| forward_cone_deg | 120.0 | deg | 전방 콘 반각 (±60°) |
| lateral_gate | 1.5 | m | 측면 오차 게이트 |
| α (alpha) | 1.0 | — | 거리 비용 가중치 |
| β (beta) | 1.2 | — | 방향 오차 가중치 |
| γ (gamma) | 0.6 | — | 측면 편차 가중치 |
| δ (delta) | 0.2 | — | 콘 크기 변화 가중치 |
| λ_side (lambda_side) | 0.5 | — | 좌우 교차 패널티 |
| branch_mode | "backbone_and_branches" | — | 브랜치 추출 전략 |
| max_branch_len | 40 | — | 최대 브랜치 길이 |
| max_chain_len | 100 | — | 최대 backbone 길이 |
| min_confidence | 0.0 | — | 신뢰도 하한 |
| resample_ds | 0.1 | m | 체인 리샘플 간격 |

### timeouts
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| perception_ms | 300 | ms | 센서 데이터 타임아웃 |

---

## 콘솔 로그 메시지 (디버그)

### 초기화 시 (1회)

| 레벨 | 메시지 | 출처 | 설명 |
|------|--------|------|------|
| INFO | `LCPlannerNode initialized (10 Hz, DirectionChainer v2 + Costmap + A*)` | 생성자 | 노드 정상 기동 확인 |
| INFO | `LC PlanningParams loaded: Costmap(10x10 res=0.15) AStar(iter=10000 tol=0.3) d_max=2.0 lat_gate=1.50` | params.hpp | yaml 파라미터 로드 결과 (값은 설정에 따라 다름) |

### 런타임 (throttle 적용 — 주기적 출력)

| 레벨 | 메시지 패턴 | 주기 | Stage | 설명 |
|------|------------|------|-------|------|
| WARN | `[Stage0] STALE — perception timeout` | 2초 | Stage 0 | perception 데이터가 `perception_ms` (300ms) 동안 갱신되지 않음. 파이프라인 중단, `/planning/status`에 `"STALE"` 발행 |
| INFO | `[Planner] OK — path:N pts, speed=X.XX m/s` | 1초 | Stage 6 | 정상 경로 생성 완료. N=waypoint 수, speed=안전 속도 |
| WARN | `[Planner] FAIL — <reason>` | 1초 | Stage 6 | 경로 생성 실패. reason: `"no_valid_path"` (A* 실패), `"curvature_exceeds_r_min"` (곡률 초과) 등 |
| INFO | `chain: L_comp=N L_bb=N L_br=N  R_comp=N R_bb=N R_br=N` | 2초 | Stage 7 | 체이닝 결과 요약. L/R=좌/우, comp=component 점 수, bb=backbone 점 수, br=branch 개수 |

### 조건부 디버그 (파라미터로 활성화)

| 레벨 | 메시지 패턴 | 활성화 조건 | 출처 | 설명 |
|------|------------|------------|------|------|
| INFO | `[LEFT] pre-resample: total=N  cones=N  lanes=N` | `chainer.debug_chainer_stats: true` | direction_chainer.cpp | 왼쪽 component의 리샘플 전 포인트 통계 (콘/차선 분류) |
| INFO | `[RIGHT] pre-resample: total=N  cones=N  lanes=N` | `chainer.debug_chainer_stats: true` | direction_chainer.cpp | 오른쪽 component의 리샘플 전 포인트 통계 |

### 로그 해석 가이드

**정상 동작 시 터미널 출력 예시:**
```
[INFO] [lc_planner_node]: LCPlannerNode initialized (10 Hz, DirectionChainer v2 + Costmap + A*)
[INFO] [lc_planner_node]: LC PlanningParams loaded: Costmap(10x10 res=0.15) AStar(iter=10000 tol=0.3) d_max=2.0 lat_gate=1.50
[INFO] [lc_planner_node]: [Planner] OK — path:47 pts, speed=1.60 m/s
[INFO] [lc_planner_node]: chain: L_comp=12 L_bb=8 L_br=2  R_comp=15 R_bb=10 R_br=3
```

**문제 상황별 대응:**

| 증상 | 로그 메시지 | 원인 | 대응 |
|------|-----------|------|------|
| 경로 없음 | `[Stage0] STALE` | perception 노드 중단 또는 토픽 미발행 | `ros2 topic hz /perception/bboxes` 로 발행 확인 |
| 경로 없음 | `[Planner] FAIL — no_valid_path` | A* 탐색 실패 (목표점 도달 불가) | costmap 시각화로 장애물 배치 확인, `max_iterations` 증가 검토 |
| 경로 불안정 | `[Planner] FAIL — curvature_exceeds_r_min` | 생성된 경로의 곡률이 차량 한계 초과 | `cone_radius`, `sigma` 조정으로 코스트맵 완화 |
| 체이닝 편향 | `chain: L_comp=0 ...` | 한쪽 경계점이 없음 | seed 파라미터(`side_seed_y`) 또는 perception 확인 |
| 속도 느림 | `speed=0.XX m/s` (v_max보다 낮음) | 곡률이 커서 횡가속도 제한 적용 | 경로 곡률 완화 또는 `a_lat_max` 상향 검토 |

---

## 빌드 & 실행

```bash
# 빌드
cd ~/ev_ws/planning && colcon build --symlink-install --packages-select chaining_costmap_ver

# 실행
ros2 launch chaining_costmap_ver chaining_costmap_ver.launch.py
```

## 빌드 구조
```
Library: chaining_costmap_chainer     (DirectionChainer)
Library: chaining_costmap_costmap     (CostmapGenerator)
Library: chaining_costmap_astar       (AStarPlanner)
Library: chaining_costmap_postprocess (PathPostprocessor)
    ↓
Component: lc_planner_component  (4개 라이브러리 링크 + 메인 노드)
    ↓
Executable: lc_planner_node_exe  (standalone 또는 composable)
```

## 소스 파일 위치
```
planning/src/chaining_costmap_ver/
├── config/
│   └── chaining_costmap_ver.yaml          ← 전체 파라미터
├── launch/
│   └── chaining_costmap_ver.launch.py     ← 실행 설정
├── include/chaining_costmap_ver/
│   ├── common/
│   │   ├── types.hpp                      ← 자료구조 정의
│   │   └── geometry.hpp                   ← 기하 유틸리티
│   ├── nodes/
│   │   └── chaining_costmap_ver_node.hpp  ← 메인 노드 헤더
│   ├── chainer/
│   │   └── direction_chainer.hpp
│   ├── costmap/
│   │   └── costmap_generator.hpp
│   ├── planner/
│   │   └── astar_planner.hpp
│   ├── postprocess/
│   │   └── path_postprocessor.hpp
│   └── safety/
│       └── safety_checker.hpp
└── src/
    ├── nodes/
    │   └── chaining_costmap_ver_node.cpp  ← 메인 노드 (8-stage pipeline)
    ├── chainer/
    │   └── direction_chainer.cpp          ← 7-step 체이닝
    ├── costmap/
    │   └── costmap_generator.cpp          ← Gaussian 코스트맵
    ├── planner/
    │   └── astar_planner.cpp              ← A* 경로탐색
    └── postprocess/
        └── path_postprocessor.cpp         ← 후처리
```
