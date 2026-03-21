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
 Stage 2: DirectionChainer  ──→  left/right backbone
    │
    ▼
 Stage 3a: CostmapGenerator  ──→  2D Gaussian cost grid
    │
    ▼
 Stage 3b: AStarPlanner  ──→  raw grid path
    │
    ▼
 Stage 5: PathPostprocessor  ──→  prune + smooth + curvature_clamp + resample + curvature_clamp + yaw
    │
    ▼
 Stage 6: SafetyChecker  ──→  curvature 검증
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

#### `/perception/bboxes` — LiDAR 장애물 바운딩 박스

- **발행자**: `make_bbox` 노드 (perception 패키지)
- **주기**: LiDAR 스캔 주기 (~10Hz)
- **좌표계**: `velodyne` (노드 내부에서 `sensor_tf` 오프셋으로 `base_link`로 변환)
- **사용 Stage**: Stage 1 (Input Parse) → ChainPoint(type=CONE)로 변환

**메시지 구조:**
```
ev_msgs/BBoxArray
├── header
│   ├── stamp        # 타임스탬프
│   └── frame_id     # "velodyne"
└── bboxes[]         # BBox 배열
    ├── position     # geometry_msgs/Point — 바운딩박스 중심 (x, y, z) [m]
    ├── size_x       # float32 — X 크기 [m]
    ├── size_y       # float32 — Y 크기 [m]
    ├── size_z       # float32 — 높이 [m]
    └── label        # int32 — DBSCAN 클러스터 ID
```

**Planning 노드에서의 사용:**
- `position.x + tf_x`, `position.y + tf_y` → base_link 좌표로 변환
- `size_x`, `size_y` → 체이닝 비용 함수 C_size에 사용
- `label` → 같은 클러스터 식별용

#### `/perception/lane_boundaries` — 카메라 차선 경계

- **발행자**: 카메라 차선 인식 노드 (perception 패키지)
- **좌표계**: `base_link` (TF 보정 불필요)
- **사용 Stage**: Stage 1 (Input Parse) → ChainPoint(type=LANE)로 변환

**메시지 구조:**
```
ev_msgs/LaneBoundaryArray
├── header
│   ├── stamp        # 타임스탬프
│   └── frame_id     # "base_link"
└── boundaries[]     # LaneBoundary 배열
    ├── header       # 개별 헤더
    └── points[]     # geometry_msgs/Point[] — 순서 정렬된 경계점 (차량에서 가까운 점부터)
```

**Planning 노드에서의 사용:**
- `points[].x`, `points[].y` → ChainPoint 좌표로 직접 사용 (이미 base_link 기준)
- `label = -1` (차선에는 클러스터 라벨 없음)

#### QoS 설정 — Best Effort, depth=1

```cpp
rclcpp::QoS qos_be(1);  // depth=1: 큐에 최대 1개만 보관
qos_be.best_effort();    // 메시지 손실 허용, 최신 데이터만 수신
```

| 항목 | 설정 | 이유 |
|------|------|------|
| Reliability | Best Effort | 인지 데이터는 실시간성이 핵심. 재전송보다 최신 데이터 우선 |
| History depth | 1 | 오래된 데이터 누적 방지 — 항상 최신 1건만 처리 |
| Durability | Volatile | 구독 전 발행된 데이터 불필요 |

> **주의**: perception 노드가 Reliable QoS로 발행하면 QoS 불일치로 연결이 안 됨.
> perception 측도 Best Effort로 맞춰야 한다.

#### Stale 판정 (Stage 0)

```
현재시각 - 마지막_수신시각 > perception_ms (기본 300ms)  →  STALE
```

- 차선 **OR** bbox 중 하나라도 fresh → 파이프라인 진행
- 둘 다 stale 또는 한 번도 수신 안 함 → `"STALE"` 상태 발행, 경로 미발행
- OR 조건 이유: 직선 구간(차선만), 장애물 구간(콘만) 등 상황별로 한쪽만 유효할 수 있음

---

### Publications (Outputs) — Core

| Topic | Message Type | QoS | 설명 |
|-------|-------------|-----|------|
| `/planning/path` | `nav_msgs::msg::Path` | BestEffort, depth=1 | 최종 후처리된 경로 (controller 입력) |
| `/planning/status` | `std_msgs::msg::String` | BestEffort, depth=1 | 플래너 상태 문자열 |

#### `/planning/path` — 최종 경로

- **생성 Stage**: Stage 5 (Postprocess) → Stage 7 (Publish)
- **좌표계**: `base_link`
- **발행 조건**: 항상 발행 (STALE일 때는 미발행)
- **구독자**: 제어기 (pure_pursuit, stanley 등)

**메시지 구조:**
```
nav_msgs/Path
├── header
│   ├── stamp        # 발행 시각
│   └── frame_id     # "base_link"
└── poses[]          # PoseStamped 배열 (등간격 waypoint)
    └── pose
        ├── position
        │   ├── x    # [m] 전방(+) / 후방(-)
        │   ├── y    # [m] 좌측(+) / 우측(-)
        │   └── z    # 항상 0.0 (2D 플래너)
        └── orientation
            └── w    # 항상 1.0 (단위 쿼터니언, yaw 정보는 별도 배열)
```

**경로 특성:**
- 등간격: `resample_ds` (기본 0.10m) 간격으로 배치
- 후처리 완료: prune → smooth → curvature_clamp → resample → curvature_clamp → yaw 적용
- 곡률 제한: `kappa_max = 1/r_min ≈ 0.461` 이하로 clamp됨
- orientation에 yaw가 포함되지 않음 — yaw는 내부 `PostprocessResult.yaw[]`에만 존재

> **참고**: `poses[].pose.orientation`은 단위 쿼터니언(w=1)로 고정.
> 제어기가 heading을 필요로 하면 인접 waypoint 간 `atan2(dy, dx)`로 직접 계산해야 함.

#### `/planning/status` — 플래너 상태

- **발행 조건**: 항상 발행 (STALE 포함)
- **구독자**: 상위 state machine, 모니터링 시스템

**상태 값:**

| 값 | 의미 | 발생 조건 |
|----|------|----------|
| `"OK"` | 정상 경로 생성 완료 | 모든 파이프라인 단계 성공 |
| `"STALE"` | 인지 데이터 타임아웃 | Stage 0에서 perception_ms 초과 |
| `"FAIL - not enough seeds"` | 시드(backbone) 생성 실패 | Stage 2에서 양쪽 backbone 모두 실패 |
| `"FAIL - no valid path"` | A* 경로 탐색 실패 | 장애물로 목표 도달 불가, 경로 점 부족 |
| `"FAIL - too short valid path"` | 경로가 너무 짧음 | 경로 총 길이 < safety.min_path_length |
| `"WARNING - curvature exceeds r_min"` | 곡률 한계 초과 (경고) | 후처리 후에도 κ > 1/r_min (경로는 발행) |

---

### Publications (Outputs) — Debug

모든 디버그 토픽은 **lazy publishing** 패턴을 사용한다:
```cpp
if (publisher->get_subscription_count() > 0) {
    // 메시지 생성 및 발행
}
```
→ RViz2에서 해당 토픽을 Add하지 않으면 CPU/메모리 소비 없음.

**QoS**: Reliable, depth=1 (RViz2가 Reliable로 구독하므로 매칭)

**발행 게이팅 — 2단계 구조:**

| 토픽 그룹 | 게이팅 조건 |
|-----------|------------|
| costmap, raw_path, obstacle_wall, curvature | lazy publish만 (구독자 있으면 항상 발행) |
| left/right_chain, seeds, local_goal | `publish_debug: true` 파라미터 **AND** lazy publish |

#### `/planning/debug/costmap` — 2D Gaussian 코스트맵

- **Type**: `nav_msgs::msg::OccupancyGrid`
- **생성 Stage**: Stage 3a (CostmapGenerator)
- **RViz2 Display**: Map

```
nav_msgs/OccupancyGrid
├── header (base_link)
├── info
│   ├── resolution   # [m/cell] 셀 크기 (기본 0.15)
│   ├── width        # 열 수
│   ├── height       # 행 수
│   └── origin       # 격자 원점 (base_link 기준)
└── data[]           # int8 배열 (0~100, costmap 값을 100으로 clamp)
```

- 값 해석: 0=자유공간, 100=최대비용(콘 중심), 중간값=Gaussian 감쇠 영역
- `obstacle_cost` (기본 100) 이상 셀은 A*에서 통과 불가

#### `/planning/debug/raw_path` — A* 원시 경로

- **Type**: `nav_msgs::msg::Path`
- **생성 Stage**: Stage 3b (AStarPlanner)
- **RViz2 Display**: Path
- 후처리 전 raw 경로 → costmap 위 8방향 그리드 탐색 결과
- 격자 단위 지그재그가 있으며, 점 간격이 불균등함

#### `/planning/debug/left_chain`, `/planning/debug/right_chain` — Backbone 체인

- **Type**: `nav_msgs::msg::Path`
- **생성 Stage**: Stage 2 (DirectionChainer)
- **게이팅**: `publish_debug: true` 필요
- **RViz2 Display**: Path
- 왼쪽/오른쪽 backbone 점들을 연결한 선
- 체이닝 알고리즘이 올바르게 좌/우 경계를 분류했는지 확인용

#### `/chaining/debug/seeds` — 시드/골 마커

- **Type**: `visualization_msgs::msg::MarkerArray`
- **생성 Stage**: Stage 2 (DirectionChainer)
- **게이팅**: `publish_debug: true` 필요
- **RViz2 Display**: MarkerArray

| 마커 | 색상 | 위치 | 크기 |
|------|------|------|------|
| 왼쪽 seed | 초록 (0,1,0) | backbone.front() | SPHERE 직경 15cm |
| 오른쪽 seed | 빨강 (1,0,0) | backbone.front() | SPHERE 직경 15cm |
| 모든 goal | 파랑 (0,0,1) | backbone.back() | SPHERE 직경 15cm |

- seed: 체이닝 시작점 (ego에서 가장 가까운 전방 포인트)
- goal: backbone 끝점 (체이닝이 도달한 가장 먼 점)

#### `/planning/debug/local_goal` — A* 목표점

- **Type**: `visualization_msgs::msg::MarkerArray`
- **생성 Stage**: Stage 3c (Goal 계산, on_timer 내부)
- **게이팅**: `publish_debug: true` 필요
- **RViz2 Display**: MarkerArray
- 노란색 SPHERE, 직경 30cm
- 양쪽 backbone 끝점의 중점, 또는 한쪽만 있으면 y×0.5 보정된 점
- costmap 경계 안쪽 1셀 마진으로 clamp됨

#### `/planning/debug/obstacle_wall` — 장애물 셀 시각화

- **Type**: `visualization_msgs::msg::MarkerArray`
- **생성 Stage**: Stage 3a (CostmapGenerator)
- **RViz2 Display**: MarkerArray
- 빨간색 CUBE_LIST (costmap 해상도 크기, 두께 0.5cm)
- `costmap[r][c] >= obstacle_cost` (기본 100)인 셀만 표시
- z=-0.01m (chain 마커 아래에 렌더링)

#### `/planning/debug/curvature` — 곡률 초과 지점

- **Type**: `visualization_msgs::msg::MarkerArray`
- **생성 Stage**: Stage 6 (SafetyChecker 후 시각화)
- **RViz2 Display**: MarkerArray
- SPHERE 직경 15cm, lifetime 0.2초
- Menger 곡률이 `kappa_limit = 1/r_min`을 초과하는 각 triplet의 중간점에 표시
- 색상 그라데이션: 노란색(약간 초과) → 빨간색(크게 초과)
  - `ratio = min((kappa/kappa_limit - 1) × 2, 1.0)`
  - `r=1.0, g=1.0-ratio, b=0.0`
- 매 프레임 DELETEALL로 이전 마커 제거

---

### 토픽 데이터 흐름 요약

```
/perception/bboxes ──────┐
  (BBoxArray, BestEffort)│  Stage 1: parse_input()
                         ├──→ ChainPoint[] (type=CONE, base_link 좌표)
/perception/lane_boundaries─┘       ↓ (type=LANE)
                         │
                    Stage 2: DirectionChainer
                         │
              ┌──────────┼──────────┐
              ▼          ▼          ▼
        left.backbone  right.backbone  unchained
              │          │          │
              └──────────┼──────────┘
                         ▼
                    Stage 3a: CostmapGenerator
                         │
                    CostmapResult ──→ /planning/debug/costmap
                         │              /planning/debug/obstacle_wall
                         ▼
                    Stage 3b: AStarPlanner
                         │
                    raw_path ──→ /planning/debug/raw_path
                         │
                    Stage 5: PathPostprocessor
                         │
                    PostprocessResult
                         │
                    Stage 6: SafetyChecker
                         │
              ┌──────────┼──────────────────────┐
              ▼          ▼                      ▼
     /planning/path   /planning/status   /planning/debug/curvature
```

### RViz2 확인 명령어

```bash
# 모든 planning 토픽 목록 확인
ros2 topic list | grep -E "planning|chaining"

# 토픽 발행 주기 확인
ros2 topic hz /planning/path

# 메시지 내용 확인
ros2 topic echo /planning/status

# 경로 waypoint 수 확인
ros2 topic echo /planning/path --field poses --once | grep -c "position:"
```

---

## 7-Stage Pipeline 상세

### Stage 0: Stale Gate
- perception_ms (기본 300ms) 이내에 데이터 수신 여부 확인
- 차선 OR bbox 중 하나라도 fresh하면 통과 (OR 조건)
- 타임아웃 시 `"STALE"` 상태 발행 후 리턴

### Stage 1: Input Parse
- BBoxArray → ChainPoint[] 변환 (sensor_tf 오프셋 적용 → base_link 좌표계)
- LaneBoundaryArray → ChainPoint[] 변환 (이미 base_link 기준)
- 좌/우 분류는 하지 않음 (Stage 2에서 seed 기반으로 결정)

### Stage 2: DirectionChainer (6-step)

| Step | 이름 | 설명 |
|------|------|------|
| 1 | Seed Selection | 좌/우 각각 ego에서 가장 가까운 전방 포인트 선택 (\|y\| > side_seed_y) |
| 2 | Graph Building | k-NN + 거리(G1) + 측면(G3) 게이트로 무방향 그래프 생성 |
| 3 | Component Extraction | 각 seed에서 BFS로 연결 컴포넌트 추출 |
| 4 | Backbone Extraction | Greedy 체이닝: 비용 함수 w'(i,j) 최소화 |
| 5 | Resampling | resample_ds 간격으로 선형 보간 |

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
  - 양쪽 backbone이 있을 때만 적용

### Stage 3b: AStarPlanner
- **8방향 그리드 탐색** (상하좌우 + 대각선)
- **시작점**: (0, 0) — 차량 위치
- **목표점 결정**:
  - 양쪽 backbone 존재 → 좌/우 끝점의 중점
  - 한쪽만 존재 → 해당 끝점의 x 그대로, y × 0.5 (중앙 방향 보정)
- **Goal clamp**: costmap 경계 안쪽 1셀 마진으로 clamp (격자 밖 goal 방지)
- **f(n) = g(n) + h(n)**
  - g(n): 누적 비용 = g(parent) + 이동비용 + costmap_cost × cost_weight
  - h(n): 유클리드 거리 (admissible heuristic)
- **장애물**: cost ≥ obstacle_cost (기본 100) → 통과 불가
- **종료**: goal_tolerance (0.3m) 이내 도달 또는 max_iterations 초과

### Stage 5: PathPostprocessor (6-step)

| Step | 이름 | 알고리즘 | 설명 |
|------|------|---------|------|
| 1 | Prune | Greedy shortcutting | 직선 구간의 불필요한 점 제거 |
| 2 | Smooth | Moving average (window=5) | 그리드 지그재그 아티팩트 제거 |
| 3 | Curvature Clamp | 중점 방향 이동 (반복 수렴) | κ > κ_max×0.95인 구간 완화 |
| 4 | Resample | 선형 보간 (ds=0.10m) | 균일 간격 waypoint 생성 |
| 5 | Curvature Clamp (2차) | 중점 방향 이동 (반복 수렴) | resample의 lerp/끝점 추가로 생긴 급커브 재보정 |
| 6 | Yaw Calc | atan2(dy, dx) | 각 waypoint의 heading 각도 |

**Curvature Clamp 상세:**
- **5% 마진**: kappa > kappa_max × 0.95 이면 보정 시작 (safety_checker 경계 WARNING 방지)
- **보정 방식**: P_i를 P_{i-1}과 P_{i+1}의 중점 방향으로 이동
- **이동 비율**: `ratio = 1.0 - 0.95*(kappa_max / kappa)`, 최대 70%
- **수렴 반복**: violations == 0이 되면 조기 종료

### Stage 6: SafetyChecker
- **Menger 곡률**: κ = 2|cross(BA, CB)| / (|AB|·|BC|·|AC|)
- **최소 회전 반경**: r_min = wheelbase / tan(δ_max) ≈ 2.17m
- **곡률 한계**: κ_limit = 1/r_min ≈ 0.461 rad/m
- **결과**: OK / FAIL - no valid path / FAIL - too short valid path / WARNING - curvature exceeds r_min
- **속도 계산 없음**: SafetyChecker는 곡률만 검사, 속도 제한은 제어기 측에서 처리

### Stage 7: Publish
- Core 토픽 항상 발행
- Debug 토픽 중 costmap/raw_path/obstacle_wall/curvature는 항상 lazy publish
- Debug 토픽 중 chainer 관련 (chains/seeds/local_goal + chain stats 로그)은 `publish_debug` 파라미터가 true일 때만 발행

---

## Parameters (chaining_costmap_ver.yaml)

### sensor_tf
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| tf_x | 0.0 | m | LiDAR → base_link 전방 오프셋 |
| tf_y | 0.0 | m | LiDAR → base_link 측면 오프셋 |
| tf_z | 0.7 | m | LiDAR 높이 |

### timeouts
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| perception_ms | 300 | ms | 센서 데이터 타임아웃 |

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
| max_chain_len | 100 | — | 최대 backbone 길이 |
| resample_ds | 0.1 | m | 체인 리샘플 간격 |
| publish_debug | true | — | chainer 디버그 마커 + 체인 통계 로그 발행 여부 |

### costmap
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| size_x | 10.0 | m | 전후 범위 |
| size_y | 10.0 | m | 좌우 범위 |
| resolution | 0.15 | m/cell | 셀 크기 (차로 1.5m = ~10셀) |
| cone_cost_max | 100.0 | — | 콘 중심 코스트 |
| lane_cost_max | 50.0 | — | 차선 경계 코스트 (콘보다 낮아서 A*가 필요시 차선 넘을 수 있음) |
| cone_radius | 1.025 | m | 콘 flat zone 반경 (0.65 + width/2 ≈ 1.025) |
| sigma | 1.0 | m | Gaussian 표준편차 |
| cost_threshold | 2.0 | — | 코스트 하한 (이하 = 0) |
| entry_wall_ego_y | 2.5 | m | entry wall 측면 오프셋 |

### astar
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| max_iterations | 10000 | — | 최대 반복 횟수 |
| goal_tolerance | 0.3 | m | 목표 도달 허용치 |
| cost_weight | 0.05 | — | 코스트맵 비용 가중치 |
| obstacle_cost | 100.0 | — | 장애물 판정 임계값 (= cone_cost_max → 콘 중심은 통과 불가) |

### postprocess
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| resample_ds | 0.10 | m | 리샘플 간격 |
| smooth_window | 5 | — | 이동평균 윈도우 크기 |
| prune_max_dev | 0.15 | m | 프루닝 최대 편차 |
| curvature_clamp_max_iter | 100 | — | 곡률 제한 최대 반복 횟수 |

### vehicle
| Parameter | 기본값 | 단위 | 설명 |
|-----------|--------|------|------|
| width | 0.79 | m | 차폭 |
| wheelbase | 0.73 | m | 축거 |
| delta_max | 0.3249 | rad | 최대 조향각 (≈18.6°, tan(18°)=0.32491) |

**파생값:**
- r_min = wheelbase / tan(delta_max) = 0.73 / tan(0.3249) ≈ **2.17m**
- κ_limit = 1 / r_min ≈ **0.461 rad/m**

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
| WARN | `[Stage0] STALE — perception timeout` | 2초 | Stage 0 | perception 데이터가 `perception_ms` (300ms) 동안 갱신되지 않음 |
| INFO | `[Planner] OK — path:N pts, kappa=X.XXX (r=X.XXm), limit=X.XXX (r_min=X.XXm, delta_max=XX.X°)` | 1초 | Stage 6 | 정상 경로 생성 완료. kappa=최대곡률, r=실제반경, limit=한계곡률 |
| WARN | `[Planner] FAIL - <reason>` | 1초 | Stage 6 | 실패 원인 포함 (reason: `"no valid path"`, `"too short valid path"` 등) |
| WARN | `[Planner] WARNING - curvature exceeds r_min — ...` | 1초 | Stage 6 | 곡률 초과 경고 (경로는 발행, kappa/r 정보 포함) |

### 조건부 디버그 (`publish_debug: true` 일 때 출력)

| 레벨 | 메시지 패턴 | 주기 | 출처 | 설명 |
|------|------------|------|------|------|
| INFO | `chain: L_comp=N L_bb=N  R_comp=N R_bb=N` | 2초 | on_timer() | 체이닝 결과 요약. L/R=좌/우, comp=component 점 수, bb=backbone 점 수 |
| INFO | `[LEFT] pre-resample: total=N  cones=N  lanes=N` | 매 호출 | direction_chainer.cpp | 왼쪽 component의 리샘플 전 포인트 통계 |
| INFO | `[RIGHT] pre-resample: total=N  cones=N  lanes=N` | 매 호출 | direction_chainer.cpp | 오른쪽 component의 리샘플 전 포인트 통계 |

### 로그 해석 가이드

**정상 동작 시 터미널 출력 예시:**
```
[INFO] [lc_planner_node]: LCPlannerNode initialized (10 Hz, DirectionChainer v2 + Costmap + A*)
[INFO] [lc_planner_node]: LC PlanningParams loaded: Costmap(10x10 res=0.15) AStar(iter=10000 tol=0.3) d_max=2.0 lat_gate=1.50
[INFO] [lc_planner_node]: [Planner] OK — path:47 pts, kappa=0.312 (r=3.21m), limit=0.461 (r_min=2.17m, delta_max=18.6°)
```

**문제 상황별 대응:**

| 증상 | 로그 메시지 | 원인 | 대응 |
|------|-----------|------|------|
| 경로 없음 | `[Stage0] STALE` | perception 노드 중단 또는 토픽 미발행 | `ros2 topic hz /perception/bboxes` 로 발행 확인 |
| 시드 부족 | `[Stage2.5] FAIL — not enough seeds` | 양쪽 backbone 모두 실패 | 인지 데이터 확인, 시드 탐색 범위(`side_seed_y`) 조정 |
| 경로 없음 | `[Planner] FAIL - no valid path` | A* 탐색 실패 (목표점 도달 불가) | costmap 시각화로 장애물 배치 확인, `max_iterations` 증가 검토 |
| 경로 너무 짧음 | `[Planner] FAIL - too short valid path` | 경로 총 길이 < min_path_length | `safety.min_path_length` 값 확인, 인지 범위 점검 |
| 경로 불안정 | `[Planner] WARNING - curvature exceeds r_min` | 생성된 경로의 곡률이 차량 한계 초과 (경로는 발행) | `curvature_clamp_max_iter` 증가, `cone_radius`/`sigma` 조정 |
| 체이닝 편향 | chain 로그에서 `L_comp=0` | 한쪽 경계점이 없음 | seed 파라미터(`side_seed_y`) 또는 perception 확인 |
| 곡률 초과 빈번 | curvature 마커 다수 표시 | 급커브 구간, costmap 과밀 | `curvature_clamp_max_iter` 증가, postprocess 파라미터 조정 |

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
│   │   ├── geometry.hpp                   ← 기하 유틸리티
│   │   ├── params.hpp                     ← 파라미터 구조체 + load()
│   │   └── debug_publish.hpp              ← 디버그 시각화 헬퍼
│   ├── nodes/
│   │   └── chaining_costmap_ver_node.hpp  ← 메인 노드 헤더 (7-stage pipeline)
│   ├── chainer/
│   │   └── direction_chainer.hpp          ← 6-step 체이닝
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
    │   └── chaining_costmap_ver_node.cpp  ← 메인 노드 (7-stage pipeline)
    ├── chainer/
    │   └── direction_chainer.cpp          ← 6-step 체이닝
    ├── costmap/
    │   └── costmap_generator.cpp          ← Gaussian 코스트맵
    ├── planner/
    │   └── astar_planner.cpp              ← A* 경로탐색
    └── postprocess/
        └── path_postprocessor.cpp         ← 후처리 (6-step)
```
