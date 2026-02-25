```markdown
# planning_mr_ver 패키지 구현 플랜

## Context
기존 `track_planning` (CDT 기반 corridor + DTR 외심 센터라인)의 대안으로, Magnetic Resistance Costmap 알고리즘 기반의 새로운 planning 패키지를 만듭니다. 여러 planning 버전을 비교하여 대회에서 최적 버전을 채용하기 위함입니다.

* **기존 패키지:** `track_planning` → `planning_cdt_ver`로 리네임 예정
* **새 패키지:** `planning_mr_ver` (Magnetic Resistance)
* **동일한 입출력 토픽:** drop-in replacement

---

## 패키지 구조

```text
planning/src/planning_mr_ver/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── planning_mr.yaml
├── launch/
│   └── planning_mr.launch.py
├── include/planning_mr_ver/
│   ├── common/
│   │   ├── types.hpp            # COPY + 수정 (CostmapResult 추가, Corridor 제거)
│   │   ├── params.hpp           # 새로 작성 (Costmap/Planner 섹션 추가)
│   │   ├── geometry.hpp         # COPY (namespace만 변경)
│   │   └── debug_publish.hpp    # COPY (namespace만 변경)
│   ├── costmap/
│   │   └── costmap_generator.hpp  # 신규
│   ├── planner/
│   │   └── magnetic_planner.hpp   # 신규
│   ├── postprocess/
│   │   └── path_postprocessor.hpp # COPY (namespace만 변경)
│   ├── safety/
│   │   └── safety_checker.hpp     # COPY (namespace만 변경)
│   └── nodes/
│       └── mr_planner_node.hpp    # 신규
└── src/
    ├── costmap/
    │   └── costmap_generator.cpp  # 신규
    ├── planner/
    │   └── magnetic_planner.cpp   # 신규
    ├── postprocess/
    │   └── path_postprocessor.cpp # COPY (namespace만 변경)
    └── nodes/
        └── mr_planner_node.cpp    # 신규

```

### 재사용 전략: COPY (namespace 변경)

독립 빌드를 위해 library dependency가 아닌 파일 복사 + namespace 변경 방식을 채택합니다. `track_planning`을 `planning_mr_ver` namespace로 일괄 변경합니다.

| 원본 파일 | 변경 내용 |
| --- | --- |
| `common/geometry.hpp` | namespace만 변경 |
| `common/debug_publish.hpp` | namespace만 변경 |
| `postprocess/path_postprocessor.hpp` | namespace만 변경 |
| `postprocess/path_postprocessor.cpp` | namespace만 변경 |
| `safety/safety_checker.hpp` | namespace만 변경 |

---

## 구현 상세

### Step 1: 스캐폴딩 — 디렉토리 + 빌드 파일 + 복사 파일

**CMakeLists.txt** — 3개 라이브러리 + 1개 컴포넌트:

* `planning_mr_costmap` ← `costmap_generator.cpp`
* `planning_mr_planner` ← `magnetic_planner.cpp`
* `planning_mr_postprocess` ← `path_postprocessor.cpp` (복사)
* `mr_planner_component` ← `mr_planner_node.cpp` (위 3개 링크)
* **의존:** `rclcpp`, `rclcpp_components`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `track_msgs`

**package.xml:** 동일 의존성, `<name>planning_mr_ver</name>`

### Step 2: common/types.hpp (수정된 복사)

* `Point2D`, `PostprocessResult`, `PlannerState` 유지
* `CorridorPolylines`, `VirtualBoundaryResult`, `CenterlineResult` 제거
* **추가:**

```cpp
struct CostmapResult {
  std::vector<double> data;   // row-major flat grid
  int rows = 0, cols = 0;
  double resolution = 0.05;
  double origin_x = -5.0;     // cell(0,0)의 world x
  double origin_y = -5.0;
  bool valid = false;
};

```

### Step 3: common/params.hpp (신규 작성)

기존 Corridor/Virtual/Centerline 섹션 전부 제거, 대신 다음 내용 추가:

```cpp
struct Costmap {
  double size_x = 10.0;          // [m] grid 전체 폭
  double size_y = 10.0;          // [m] grid 전체 높이
  double resolution = 0.05;      // [m/cell] (200x200 = 40,000 cells)
  double cone_cost_max = 100.0;  // 콘 최대 cost
  double lane_cost_max = 50.0;   // 차선 최대 cost
  double alpha = 2.0;            // 감쇠율: cost = max / (1 + α·d²)
  double cost_threshold = 2.0;   // cutoff: 미만이면 0 처리
} costmap;

struct Planner {
  double search_radius = 0.30;   // [m] 전방 180° 탐색 반경
  int max_steps = 200;           // 최대 경로 포인트 수
  double heading_init_x = 1.0;
  double heading_init_y = 0.0;
} planner;

```

* `Vehicle`, `Safety`, `Speed`, `Postprocess`, `Timeouts` 섹션은 기존과 동일하게 유지.

### Step 4: costmap/costmap_generator (신규 — 핵심 알고리즘 1)

```cpp
class CostmapGenerator {
public:
  CostmapResult generate(
    const std::vector<Point2D>& cones,
    const std::vector<Point2D>& lanes,
    const PlanningParams& params);

private:
  // 단일 source의 1/r² 자력을 grid에 적용 (MAX override)
  static void apply_source(grid, source, cost_max, alpha, threshold);

  // effective_radius: sqrt((cost_max/threshold - 1) / alpha)
  static double effective_radius(cost_max, alpha, threshold);
};

```

**알고리즘:**

1. `rows×cols` grid 할당, 0으로 초기화
2. 각 cone source: `effective_radius` 범위 내 cell만 순회
* `cost = cost_max / (1 + alpha * d²)`
* `if cost < threshold` → skip
* `grid[cell] = max(grid[cell], cost)` (MAX override)


3. 각 lane source: 동일 (`cost_max=50`)
4. return `CostmapResult`

**성능:** `alpha=2.0`, `threshold=2.0` 기준

* cone `r_eff` ≈ 5.0m, lane `r_eff` ≈ 3.5m
* 콘 20개 + 차선점 50개 = 10Hz에서 충분

### Step 5: planner/magnetic_planner (신규 — 핵심 알고리즘 2)

```cpp
class MagneticPlanner {
public:
  std::vector<Point2D> plan(
    const CostmapResult& costmap,
    const PlanningParams& params);

private:
  static bool world_to_grid(wx, wy, ..., &row, &col);
  static Point2D grid_to_world(row, col, ...);
  static Point2D find_best_forward_cell(
    costmap, current_pos, heading, params, &found);
};

```

**알고리즘:**

1. ego(0,0), heading(1,0)에서 시작
2. 반복 (`max_steps`회):
* `search_radius` 내 cell 중 전방 180° (`dot(heading, dir) > 0`)
* 가장 낮은 cost의 cell 선택
* `heading = normalize(new_pos - old_pos)`로 업데이트
* grid 경계 도달 시 종료


3. return `raw_path` (`vector<Point2D>`)

* **Oscillation 방지:** cost 동률 시 heading 방향에 더 가까운 cell 우선 (tie-breaker)

### Step 6: nodes/mr_planner_node (신규)

기존 `local_planner_node`의 scaffolding 재활용:

* **동일하게 유지:**
* Subscriptions: `/perception/lane_boundaries`, `/perception/cones` (UniquePtr, QoS(1))
* Publishers: `/planning/path`, `/planning/status`
* Timer: 10Hz wall_timer
* `check_stale()`: OR 로직 동일
* ComposableNode 등록: `RCLCPP_COMPONENTS_REGISTER_NODE`


* **변경:**
* `parse_lanes()` → 좌/우 분리 없이 모든 lane point를 단일 벡터로 수집
* `parse_cones()` → 좌/우 분리 없이 모든 cone을 단일 벡터로 수집
* Pipeline modules: `CostmapGenerator` + `MagneticPlanner` + `PathPostprocessor` (3개)
* Debug publishers:
* `/planning/debug/costmap` (`nav_msgs/OccupancyGrid`) — 신규
* `/planning/debug/raw_path` (`nav_msgs/Path`) — 신규




* **제거:** `w_hat_`, corridor/centerline debug topics
* **`on_timer()` 파이프라인 (7 Stage):**
* Stage 0: Stale Gate (동일)
* Stage 1: Input Parse (단순화: L/R 분리 안 함)
* Stage 2: Costmap Generator (신규)
* Stage 3: Magnetic Planner (신규)
* Stage 4: Postprocessor (동일: prune→smooth→resample→yaw)
* Stage 5: Safety Check (동일: Menger 곡률 + 속도제한)
* Stage 6: Publish (동일 + OccupancyGrid debug)



### Step 7: config/planning_mr.yaml

```yaml
mr_planner_node:
  ros__parameters:
    costmap:
      size_x: 10.0
      size_y: 10.0
      resolution: 0.05
      cone_cost_max: 100.0
      lane_cost_max: 50.0
      alpha: 2.0
      cost_threshold: 2.0
    planner:
      search_radius: 0.30
      max_steps: 200
      heading_init_x: 1.0
      heading_init_y: 0.0
    vehicle: { width: 0.50, wheelbase: 0.87, delta_max: 0.314 }
    safety: { margin: 0.10 }
    speed: { v_max: 1.60, a_lat_max: 2.0 }
    postprocess: { resample_ds: 0.10, smooth_window: 5, prune_max_dev: 0.15 }
    timeouts: { perception_ms: 300 }

```

### Step 8: launch/planning_mr.launch.py

기존 `planning.launch.py`와 동일 패턴:

* `get_package_share_directory('planning_mr_ver')`
* `yaml.safe_load` → `mr_planner_node/ros__parameters`
* `ComposableNodeContainer` + `ComposableNode(plugin='planning_mr_ver::MRPlannerNode')`
* `use_intra_process_comms: True`

---

## 구현 순서

| 작업 | 파일 수 | 비고 |
| --- | --- | --- |
| 1. 디렉토리 생성 + `CMakeLists.txt` + `package.xml` | 2 | 빌드 뼈대 |
| 2. 복사 파일 5개 (namespace 변경) | 5 | `geometry`, `debug_publish`, `postprocessor`, `safety_checker` |
| 3. `types.hpp` + `params.hpp` | 2 | `CostmapResult` 추가, MR params |
| 4. `costmap_generator.hpp/.cpp` | 2 | 핵심 알고리즘 1 |
| 5. `magnetic_planner.hpp/.cpp` | 2 | 핵심 알고리즘 2 |
| 6. `mr_planner_node.hpp/.cpp` | 2 | 노드 배선 |
| 7. `planning_mr.yaml` + launch file | 2 | 설정 + 실행 |
| 8. `colcon build` 검증 | - | 빌드 성공 확인 |
| **총계** | **17개** | 파일 생성 |

---

## 검증 방법

1. `cd ~/ev_ws/planning && colcon build --packages-select planning_mr_ver` → 빌드 성공 확인
2. `ros2 launch planning_mr_ver planning_mr.launch.py` → 노드 실행
3. `rosbag` 재생하여 `/planning/path` 토픽 출력 확인
4. `RViz2`에서 `/planning/debug/costmap` (`OccupancyGrid`) + `/planning/debug/raw_path` (`Path`) 시각화

## 토픽 인터페이스 (기존과 동일 = drop-in)

| 방향 | 토픽 | 타입 | CDT와 동일 여부 |
| --- | --- | --- | --- |
| Sub | `/perception/lane_boundaries` | `LaneBoundaryArray` | O |
| Sub | `/perception/cones` | `ConeArray` | O |
| Pub | `/planning/path` | `nav_msgs/Path` | O |
| Pub | `/planning/status` | `PlannerStatus` | O |
| Pub (debug) | `/planning/debug/costmap` | `OccupancyGrid` | 신규 |
| Pub (debug) | `/planning/debug/raw_path` | `nav_msgs/Path` | 신규 |


