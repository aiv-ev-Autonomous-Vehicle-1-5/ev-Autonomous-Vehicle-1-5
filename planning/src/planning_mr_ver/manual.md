# planning_mr_ver 패키지 파이프라인 설명서

## 목차
1. [패키지 개요](#1-패키지-개요)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [빌드 & 실행 구조](#3-빌드--실행-구조)
4. [공통 자료구조 (common/)](#4-공통-자료구조-common)
5. [파이프라인 전체 흐름](#5-파이프라인-전체-흐름)
6. [Stage 0: Stale Gate — 입력 타임아웃 검사](#6-stage-0-stale-gate--입력-타임아웃-검사)
7. [Stage 1: Input Parse — 인식 데이터 파싱](#7-stage-1-input-parse--인식-데이터-파싱)
8. [Stage 2: Costmap Generation — 자기 저항 코스트맵 생성](#8-stage-2-costmap-generation--자기-저항-코스트맵-생성)
9. [Stage 3: Magnetic Planner — Greedy 전진 경로 탐색](#9-stage-3-magnetic-planner--greedy-전진-경로-탐색)
10. [Stage 4: Postprocess — 경로 후처리](#10-stage-4-postprocess--경로-후처리)
11. [Stage 5: Safety Check — 안전 검사 & 속도 결정](#11-stage-5-safety-check--안전-검사--속도-결정)
12. [Stage 6: Publish — 결과 퍼블리시](#12-stage-6-publish--결과-퍼블리시)
13. [파라미터 일람표](#13-파라미터-일람표)
14. [ROS 2 토픽 인터페이스](#14-ros-2-토픽-인터페이스)

---

## 1. 패키지 개요

`planning_mr_ver`은 **Magnetic Resistance(자기 저항) 모델**에 기반한 로컬 경로 계획 패키지다.

핵심 아이디어는 **장애물(콘, 차선 경계)을 자석의 S극으로 모델링**하여, 자력이 강한 영역(위험 영역)을 피하고 자력이 약한 영역(안전 영역)으로 경로를 생성하는 것이다.

```
   콘(S극, cost=100)       차선(S극, cost=50)
        ●══════╗               ·─·─·─·─·
        ║ flat ║              /  decay  \
        ║ zone ║             /   zone    \
        ╚══════╝
         ↓ 1/r² 감쇠          ↓ 1/r² 감쇠
```

10Hz 타이머 콜백으로 구동되며, 한 번의 콜백에서 다음 6단계 파이프라인이 **순차적으로** 실행된다:

```
Stale Gate → Input Parse → Costmap → Planner → Postprocess → Safety → Publish
```

---

## 2. 디렉토리 구조

```
planning_mr_ver/
├── CMakeLists.txt                          # 빌드 설정
├── package.xml                             # 패키지 메타데이터
├── config/
│   └── planning_mr.yaml                    # 전체 파라미터
├── launch/
│   └── planning_mr.launch.py               # ComposableNode launch
├── include/planning_mr_ver/
│   ├── common/
│   │   ├── types.hpp                       # Point2D, CostmapResult, PostprocessResult, PlannerState
│   │   ├── params.hpp                      # PlanningParams 구조체 + load()
│   │   ├── geometry.hpp                    # 기하학 유틸리티 (dist, normalize, resample 등)
│   │   └── debug_publish.hpp               # to_path_msg(), to_bool_msg() 변환 헬퍼
│   ├── costmap/
│   │   └── costmap_generator.hpp           # CostmapGenerator 클래스 선언
│   ├── planner/
│   │   └── magnetic_planner.hpp            # MagneticPlanner 클래스 선언
│   ├── postprocess/
│   │   └── path_postprocessor.hpp          # PathPostprocessor 클래스 선언
│   ├── safety/
│   │   └── safety_checker.hpp              # safety_checker 네임스페이스 (header-only)
│   └── nodes/
│       └── mr_planner_node.hpp             # MRPlannerNode 클래스 선언
└── src/
    ├── nodes/
    │   └── mr_planner_node.cpp             # 노드 구현 (타이머 콜백, 파이프라인 조립)
    ├── costmap/
    │   └── costmap_generator.cpp           # 코스트맵 생성 구현
    ├── planner/
    │   └── magnetic_planner.cpp            # 경로 탐색 구현
    └── postprocess/
        └── path_postprocessor.cpp          # 경로 후처리 구현
```

---

## 3. 빌드 & 실행 구조

### 3.1 빌드 구성 (CMakeLists.txt)

패키지는 **3개의 공유 라이브러리 + 1개의 ComposableNode 컴포넌트**로 구성된다.

```cmake
# 라이브러리 1: 코스트맵 생성
add_library(planning_mr_costmap SHARED
  src/costmap/costmap_generator.cpp
)

# 라이브러리 2: 경로 탐색
add_library(planning_mr_planner SHARED
  src/planner/magnetic_planner.cpp
)

# 라이브러리 3: 경로 후처리
add_library(planning_mr_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)

# 컴포넌트: 위 3개 라이브러리를 링크하여 하나의 노드로 조립
add_library(mr_planner_component SHARED
  src/nodes/mr_planner_node.cpp
)
target_link_libraries(mr_planner_component
  planning_mr_costmap
  planning_mr_planner
  planning_mr_postprocess
)
```

> `mr_planner_component`가 3개 모듈 라이브러리를 **링크**하는 구조. 모듈별 독립 빌드가 가능하고, 단위 테스트도 모듈 단위로 할 수 있다.

컴포넌트 등록:

```cmake
rclcpp_components_register_node(mr_planner_component
  PLUGIN "planning_mr_ver::MRPlannerNode"
  EXECUTABLE mr_planner_node_exe
)
```

> 이 매크로가 **2가지**를 동시에 생성한다:
> 1. `component_container`에 동적 로드할 수 있는 공유 라이브러리 (`mr_planner_component.so`)
> 2. 단독 실행 가능한 바이너리 (`mr_planner_node_exe`)

### 3.2 Launch 파일

`launch/planning_mr.launch.py`에서 ComposableNodeContainer를 통해 실행한다:

```python
container = ComposableNodeContainer(
    name='planning_mr_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='planning_mr_ver',
            plugin='planning_mr_ver::MRPlannerNode',
            name='mr_planner_node',
            parameters=[params],
            extra_arguments=[{'use_intra_process_comms': True}],  # ← Zero-copy 통신
        ),
    ],
    output='both',
)
```

핵심 포인트:
- `use_intra_process_comms: True` — 같은 컨테이너 내 노드 간 **Zero-copy** 데이터 전달 활성화
- YAML 파일을 직접 파싱하여 `parameters`로 전달 (launch에서 `ros__parameters` 하위만 추출)

실행 명령:
```bash
ros2 launch planning_mr_ver planning_mr.launch.py
```

---

## 4. 공통 자료구조 (common/)

### 4.1 types.hpp — 핵심 타입 정의

파이프라인 전체에서 공유하는 4개의 자료구조:

```cpp
// (types.hpp:25-29)
struct Point2D
{
  double x = 0.0;  // X 좌표 [m] — 전방(+), 후방(-)
  double y = 0.0;  // Y 좌표 [m] — 좌측(+), 우측(-)
};
```

> ego 차량의 `base_link` 좌표계 기준. 모든 입력/출력 좌표가 이 구조체를 사용한다.

```cpp
// (types.hpp:46-55)
struct CostmapResult
{
  std::vector<double> data;   // row-major flat grid: data[row * cols + col]
  int rows = 0;               // 그리드 행 수
  int cols = 0;               // 그리드 열 수
  double resolution = 0.05;   // 셀 크기 [m/cell]
  double origin_x = -5.0;     // cell(0,0)의 월드 X 좌표
  double origin_y = -5.0;     // cell(0,0)의 월드 Y 좌표
  bool valid = false;
};
```

> 기본 설정 기준 200×200 = **40,000 cells**의 2D 그리드. `data`가 1차원 배열이므로 `data[row * cols + col]`로 2D 접근한다.

```cpp
// (types.hpp:63-68)
struct PostprocessResult
{
  std::vector<Point2D> path;  // 후처리된 경로 점 목록
  std::vector<double> yaw;    // 각 점의 heading 각도 [rad]
  bool valid = false;
};
```

```cpp
// (types.hpp:76-82)
enum class PlannerState : uint8_t
{
  OK = 0,          // 정상 — 경로 추종 가능
  STOP = 1,        // 정지 — 유효한 경로 없음
  INFEASIBLE = 2,  // 실행 불가 — 곡률이 최소 회전반경 초과
  STALE = 3        // 타임아웃 — perception 데이터 수신 안 됨
};
```

### 4.2 params.hpp — 파라미터 구조체

모든 파라미터를 **중첩 struct**로 관리한다:

```cpp
// (params.hpp:10-136)
struct PlanningParams
{
  struct Costmap { ... } costmap;
  struct Planner { ... } planner;
  struct Vehicle {
    double width = 0.50;
    double wheelbase = 0.87;
    double delta_max = 0.314;
    double r_min() const { return wheelbase / std::tan(delta_max); }  // 최소 회전 반경
  } vehicle;
  struct Safety { ... } safety;
  struct Speed { ... } speed;
  struct Postprocess { ... } postprocess;
  struct Timeouts { ... } timeouts;

  void load(rclcpp::Node * node) { ... }
};
```

`load()` 메서드는 ROS 2 파라미터 서버에서 값을 읽는 람다를 사용한다:

```cpp
// (params.hpp:89-92)
auto p = [&](const std::string & name, auto default_val) {
  node->declare_parameter(name, rclcpp::ParameterValue(default_val));
  return node->get_parameter(name).get_value<decltype(default_val)>();
};
```

> `declare_parameter`로 선언과 동시에 기본값을 지정하고, 바로 `get_parameter`로 현재 값을 가져온다. YAML 파일에서 값이 오버라이드되면 그 값이 사용된다.

### 4.3 geometry.hpp — 기하학 유틸리티

경로 처리에 필요한 수학 함수들 (header-only):

| 함수 | 역할 |
|------|------|
| `dist(a, b)` | 두 점 사이 유클리드 거리 |
| `normalize(v)` | 단위 벡터로 정규화 |
| `cross2(a, b)` | 2D 외적 (a.x*b.y - a.y*b.x) |
| `wrap_pi(angle)` | 각도를 [-π, +π] 범위로 정규화 |
| `heading(v)` | 벡터의 heading 각도 (atan2) |
| `lerp(a, b, t)` | 선형 보간 |
| `resample_polyline(pts, ds)` | 폴리라인을 일정 간격 `ds`로 리샘플링 |
| `polyline_tangents(pts)` | 각 점의 접선 벡터 계산 |

특히 `resample_polyline`은 후처리에서 핵심적으로 사용된다:

```cpp
// (geometry.hpp)
inline std::vector<Point2D> resample_polyline(
  const std::vector<Point2D> & pts, double ds)
{
  // 누적 거리 방식으로 ds 간격마다 보간점 생성
  // 마지막 점과의 거리가 ds*0.1 이상이면 끝점도 추가
}
```

### 4.4 debug_publish.hpp — 디버그 퍼블리시 헬퍼

내부 자료구조를 ROS 메시지로 변환하는 유틸리티:

```cpp
// (debug_publish.hpp)
inline nav_msgs::msg::Path to_path_msg(
  const std::vector<Point2D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  // Point2D 벡터 → nav_msgs::Path 변환
  // 각 점을 PoseStamped로 감싸서 poses 배열에 추가
}
```

---

## 5. 파이프라인 전체 흐름

`MRPlannerNode`의 생성자에서 **10Hz 타이머**가 등록되고, 매 100ms마다 `on_timer()` 콜백이 호출된다:

```cpp
// (mr_planner_node.cpp:48-51)
timer_ = create_wall_timer(
  std::chrono::milliseconds(100),
  std::bind(&MRPlannerNode::on_timer, this));
```

`on_timer()` 내부에서 6개 스테이지가 순차 실행된다:

```cpp
// (mr_planner_node.cpp:93-169)
void MRPlannerNode::on_timer()
{
  // Stage 0: Stale Gate       — 입력 데이터 유효성 검사
  // Stage 1: Input Parse      — 콘/차선 데이터 파싱
  // Stage 2: Costmap Generation — MR 코스트맵 생성
  // Stage 3: Magnetic Planner  — Greedy 경로 탐색
  // Stage 4: Postprocess      — 가지치기 → 평활화 → 리샘플링
  // Stage 5: Safety Check     — 곡률 검사 & 속도 결정
  // Stage 6: Publish          — 결과 퍼블리시
}
```

전체 데이터 흐름도:

```
  /perception/lane_boundaries ─┐
                                ├──→ [Stage 0: Stale Gate]
  /perception/cones ───────────┘          │
                                          │ (300ms 이내?)
                                          ▼
                                    [Stage 1: Input Parse]
                                          │
                                    parse_cones()  → vector<Point2D>
                                    parse_lanes()  → vector<Point2D>
                                          │
                                          ▼
                                    [Stage 2: Costmap Generation]
                                          │
                                    CostmapGenerator::generate()
                                    → CostmapResult (200×200 grid)
                                          │
                                          ▼
                                    [Stage 3: Magnetic Planner]
                                          │
                                    MagneticPlanner::plan()
                                    → vector<Point2D> (raw_path)
                                          │
                                          ▼
                                    [Stage 4: Postprocess]
                                          │
                                    PathPostprocessor::process()
                                    → PostprocessResult (path + yaw)
                                          │
                                          ▼
                                    [Stage 5: Safety Check]
                                          │
                                    safety_checker::check()
                                    → SafetyResult (state + speed)
                                          │
                                          ▼
                                    [Stage 6: Publish]
                                          │
                    ┌─────────────────┬────┴────────────────┐
                    ▼                 ▼                      ▼
              /planning/path   /planning/status    /planning/debug/*
```

---

## 6. Stage 0: Stale Gate — 입력 타임아웃 검사

**목적**: perception 데이터가 너무 오래되었으면 경로 계획을 건너뛰고 STALE 상태를 퍼블리시한다.

```cpp
// (mr_planner_node.cpp:76-91)
bool MRPlannerNode::check_stale() const
{
  const auto t = now();
  bool have_perception = false;

  if (last_lanes_) {
    const double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;  // ns → ms 변환
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_cones_) {
    const double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }

  return !have_perception;  // 둘 다 타임아웃이면 stale
}
```

동작 방식:
1. 현재 시간과 마지막 수신 시간의 차이를 ms 단위로 계산
2. **차선 또는 콘 중 하나라도** `perception_ms`(기본 300ms) 이내이면 유효
3. **둘 다** 타임아웃이면 stale → 파이프라인 중단

stale 상태일 때의 처리:

```cpp
// (mr_planner_node.cpp:98-107)
if (check_stale()) {
  auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
  status_msg->header.stamp = stamp;
  status_msg->header.frame_id = frame_id;
  status_msg->status = track_msgs::msg::PlannerStatus::STALE;
  status_msg->reason = "input_stale";
  pub_status_->publish(std::move(status_msg));
  return;  // ← 여기서 즉시 반환, 이후 스테이지 실행하지 않음
}
```

> **설계 의도**: perception 노드가 죽거나 네트워크 장애가 발생했을 때 마지막으로 받은 오래된 데이터로 잘못된 경로를 계획하는 것을 방지한다.

---

## 7. Stage 1: Input Parse — 인식 데이터 파싱

**목적**: ROS 메시지 형태의 인식 데이터를 파이프라인 내부 자료구조(`Point2D` 벡터)로 변환한다.

### 7.1 구독 설정

```cpp
// (mr_planner_node.cpp:22-34)
sub_lanes_ = create_subscription<track_msgs::msg::LaneBoundaryArray>(
  "/perception/lane_boundaries", rclcpp::QoS(1),
  [this](track_msgs::msg::LaneBoundaryArray::UniquePtr msg) {
    stamp_lanes_ = now();           // 수신 시간 기록
    last_lanes_ = std::move(msg);   // UniquePtr로 Zero-copy 이동
  });

sub_cones_ = create_subscription<track_msgs::msg::ConeArray>(
  "/perception/cones", rclcpp::QoS(1),
  [this](track_msgs::msg::ConeArray::UniquePtr msg) {
    stamp_cones_ = now();
    last_cones_ = std::move(msg);
  });
```

> `UniquePtr`로 수신하는 이유: `use_intra_process_comms: True`가 활성화된 환경에서 **메모리 복사 없이** 메시지 소유권을 이동(Zero-copy)할 수 있다.

### 7.2 데이터 파싱

```cpp
// (mr_planner_node.cpp:56-74)
void MRPlannerNode::parse_lanes(
  std::vector<Point2D> & all_lane_pts) const
{
  if (!last_lanes_) return;
  for (const auto & b : last_lanes_->boundaries) {   // 각 차선 경계
    for (const auto & p : b.points) {                 // 경계의 각 점
      all_lane_pts.push_back({p.x, p.y});             // 3D → 2D 변환
    }
  }
}

void MRPlannerNode::parse_cones(
  std::vector<Point2D> & all_cones) const
{
  if (!last_cones_) return;
  for (const auto & c : last_cones_->cones) {
    all_cones.push_back({c.position.x, c.position.y});
  }
}
```

> **특징**: 코스트맵에서는 좌/우 차선을 구분할 필요가 없다. 모든 차선 경계점과 콘을 **하나의 벡터로 통합**하여 처리한다. 이것이 기존 `track_planning`(CDT 기반)과의 핵심적인 구조 차이점이다.

실제 호출:

```cpp
// (mr_planner_node.cpp:109-112)
std::vector<Point2D> all_cones, all_lane_pts;
parse_cones(all_cones);
parse_lanes(all_lane_pts);
```

---

## 8. Stage 2: Costmap Generation — 자기 저항 코스트맵 생성

**목적**: 콘과 차선 경계점을 "자석의 S극"으로 모델링하여, ego 중심 2D 그리드에 자력(cost)을 기록한다.

### 8.1 핵심 수식

```
d <= inner_radius:
    cost = cost_max                           (flat zone — 충돌 영역)

d > inner_radius:
    d_eff = d - inner_radius
    cost = cost_max / (1 + α · d_eff²)       (1/r² 감쇠)

cost < threshold:
    cost = 0                                  (cutoff — 무한 꼬리 제거)

grid[cell] = max(grid[cell], cost)            (MAX override)
```

시각화:

```
cost ^
100 ─── |████████|                     ← flat zone (d ≤ 0.65m, 콘 반지름 내부)
        |████████|╲
 50 ─── |        |  ╲                  ← 1/r² decay zone
        |        |    ╲
  2 ─── |        |      ╲_________    ← threshold cutoff (이 이하는 0 취급)
  0 ─── |--------|-------|---------|→ d [m]
        0      0.65    ~5.6m
            inner_r   r_total
```

### 8.2 generate() — 메인 진입점

```cpp
// (costmap_generator.cpp:190-235)
CostmapResult CostmapGenerator::generate(
  const std::vector<Point2D> & cones,
  const std::vector<Point2D> & lanes,
  const PlanningParams & params)
{
  CostmapResult result;
  const auto & cm = params.costmap;

  // 그리드 크기 계산: 10.0 / 0.05 = 200
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));

  // 원점 = ego 좌하단: (-5.0, -5.0)
  result.origin_x = -cm.size_x / 2.0;
  result.origin_y = -cm.size_y / 2.0;

  // 모든 셀을 0으로 초기화 (장애물 없는 자유 공간)
  result.data.assign(result.rows * result.cols, 0.0);

  // 콘 source: inner_radius = cone_radius (flat zone 있음)
  for (const auto & cone : cones) {
    apply_source(... cone, cm.cone_cost_max, ..., cm.cone_radius);
  }

  // 차선 source: inner_radius = 0 (flat zone 없음, 중심부터 바로 감쇠)
  for (const auto & lane_pt : lanes) {
    apply_source(... lane_pt, cm.lane_cost_max, ..., 0.0);
  }

  result.valid = true;
  return result;
}
```

그리드 좌표계:

```
  (-5, +5) ─────────────────── (+5, +5)
    │                              │
    │         ego(0,0)             │
    │            ★                 │
    │                              │
  (-5, -5) ─────────────────── (+5, -5)
  = origin                    = origin + size
```

### 8.3 effective_radius() — 유효 영향 반경

감쇠가 threshold까지 내려가는 거리를 사전 계산하여, 전체 그리드를 순회하지 않고 **bounding box 범위만 처리**한다:

```cpp
// (costmap_generator.cpp:39-51)
double CostmapGenerator::effective_radius(
  double cost_max, double alpha, double threshold)
{
  if (threshold <= 0.0 || alpha <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;
  return std::sqrt((ratio - 1.0) / alpha);
}
```

수학 유도:
```
cost_max / (1 + α·d²) = threshold
→ 1 + α·d² = cost_max / threshold
→ d² = (cost_max/threshold - 1) / α
→ d = sqrt((cost_max/threshold - 1) / α)
```

기본 파라미터 대입:
- **콘**: `sqrt((100/2 - 1) / 2) = sqrt(49/2) ≈ 4.95m` → 총 영향 반경 = 0.65 + 4.95 = **5.60m**
- **차선**: `sqrt((50/2 - 1) / 2) = sqrt(24/2) ≈ 3.46m` → 총 영향 반경 = 0 + 3.46 = **3.46m**

### 8.4 apply_source() — 단일 장애물의 자력 기록

```cpp
// (costmap_generator.cpp:96-164)
void CostmapGenerator::apply_source(
  std::vector<double> & grid,
  int rows, int cols,
  double resolution, double origin_x, double origin_y,
  const Point2D & source,
  double cost_max, double alpha, double threshold,
  double inner_radius)
{
  // [Step 1] 총 영향 반경 계산
  double r_decay = effective_radius(cost_max, alpha, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // [Step 2] source → grid 좌표 변환
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // [Step 3] bounding box 클램프
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  // [Step 4] bounding box 내 각 셀 순회
  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      // 셀 중심의 world 좌표 (0.5 offset → 셀 정중앙)
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        cost = cost_max;               // flat zone
      } else {
        double d_eff = d - inner_radius;
        cost = cost_max / (1.0 + alpha * d_eff * d_eff);  // 1/r² 감쇠
        if (cost < threshold) continue;  // cutoff
      }

      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;              // MAX override
      }
    }
  }
}
```

**성능 최적화 포인트**:
- bounding box로 탐색 범위를 제한 → 전체 40,000 셀 순회 대비 **대폭 성능 향상**
- MAX override: 여러 source가 겹칠 때 **가장 강한 자력이 지배** (자석 비유에 충실)

### 8.5 콘 vs 차선의 차이

| 속성 | 콘 (Cone) | 차선 (Lane) |
|------|-----------|-------------|
| cost_max | 100 (최대 위험) | 50 (중간 위험) |
| inner_radius | 0.65m (물리적 크기 반영) | 0.0m (두께 없는 점) |
| flat zone | 있음 (클러스터 내부) | 없음 (중심부터 감쇠) |
| 총 영향 반경 | ~5.60m | ~3.46m |

> 콘은 물리적 크기(직경 500mm)가 있으므로 반지름 0.65m 내부를 flat zone으로 설정하여 **충돌 영역을 명확히 표현**한다. 차선은 얇은 선이므로 flat zone 없이 중심에서 바로 감쇠한다.

---

## 9. Stage 3: Magnetic Planner — Greedy 전진 경로 탐색

**목적**: 코스트맵 위에서 ego(0,0)부터 시작하여, 매 스텝마다 **전방 180도 범위에서 cost가 가장 낮은 셀**로 이동하며 경로를 생성한다.

### 9.1 plan() — 경로 탐색 메인 루프

```cpp
// (magnetic_planner.cpp:97-141)
std::vector<Point2D> MagneticPlanner::plan(
  const CostmapResult & costmap,
  const PlanningParams & params)
{
  std::vector<Point2D> raw_path;
  if (!costmap.valid) return raw_path;

  // 시작점: ego 위치 (0,0)
  Point2D current_pos{0.0, 0.0};

  // 초기 heading: (1,0) → +x 방향 (전방)
  Point2D hdg = normalize(
    Point2D{params.planner.heading_init_x, params.planner.heading_init_y});

  raw_path.push_back(current_pos);

  for (int step = 0; step < params.planner.max_steps; ++step) {
    bool found = false;
    Point2D next_pos = find_best_forward_cell(
      costmap, current_pos, hdg, params, found);

    if (!found) break;  // 전방에 갈 수 있는 셀이 없으면 종료

    // heading 업데이트: 이동 방향으로 갱신
    Point2D new_hdg = normalize(next_pos - current_pos);
    if (norm(new_hdg) < 1e-6) break;

    hdg = new_hdg;
    current_pos = next_pos;
    raw_path.push_back(current_pos);

    // grid 경계 근처 도달 시 종료 (margin 2 cells)
    int r, c;
    if (!world_to_grid(current_pos.x, current_pos.y, ...)) break;
    if (r <= 1 || r >= costmap.rows - 2 ||
        c <= 1 || c >= costmap.cols - 2) {
      break;
    }
  }

  return raw_path;
}
```

종료 조건:
1. **`max_steps`(200) 도달** — 최대 경로 길이 제한
2. **`found == false`** — 전방에 이동 가능한 셀이 없음
3. **grid 경계 도달** — 경계에서 2셀 이내에 도달하면 종료

### 9.2 find_best_forward_cell() — 전방 최소 cost 셀 탐색

이 함수가 플래너의 **핵심 알고리즘**이다:

```cpp
// (magnetic_planner.cpp:33-95)
Point2D MagneticPlanner::find_best_forward_cell(
  const CostmapResult & costmap,
  const Point2D & current_pos,
  const Point2D & hdg,
  const PlanningParams & params,
  bool & found)
{
  found = false;
  double best_cost = std::numeric_limits<double>::max();
  double best_dot = -1.0;  // tie-breaker: heading 정렬도
  Point2D best_pos{0.0, 0.0};

  const double r = params.planner.search_radius;  // 0.30m
  const double r_sq = r * r;
  int r_cells = static_cast<int>(std::ceil(r / costmap.resolution));

  // 현재 위치를 grid 좌표로 변환
  int cur_row, cur_col;
  if (!world_to_grid(...)) return best_pos;

  // bounding box 내 셀 순회
  for (int row = row_min; row <= row_max; ++row) {
    for (int col = col_min; col <= col_max; ++col) {
      Point2D cell_pos = grid_to_world(row, col, ...);

      double dx = cell_pos.x - current_pos.x;
      double dy = cell_pos.y - current_pos.y;
      double d_sq = dx * dx + dy * dy;

      // 자기 자신 & 탐색 반경 밖 제외
      if (d_sq < 1e-12 || d_sq > r_sq) continue;

      // ★ 전방 180° 체크: dot(heading, direction) > 0
      double dot_val = hdg.x * dx + hdg.y * dy;
      if (dot_val <= 0.0) continue;  // 후방 셀은 무시

      double cost = costmap.data[row * costmap.cols + col];

      // heading 정렬도: cos(angle) = dot / distance
      double d = std::sqrt(d_sq);
      double alignment = dot_val / d;

      // ★ 선택 기준: 최소 cost, 동률이면 heading에 더 정렬된 셀
      if (cost < best_cost || (cost == best_cost && alignment > best_dot)) {
        best_cost = cost;
        best_dot = alignment;
        best_pos = cell_pos;
        found = true;
      }
    }
  }

  return best_pos;
}
```

탐색 과정 시각화:

```
          search_radius = 0.30m
              ┌─────────┐
              │    ○     │  ← 전방 180° 내 셀들만 후보
              │   /|\    │
              │  / | \   │
      ────────┼─/──★──\──┼──────── heading 방향 →
              │ (현재위치) │
              │  후방: 무시 │
              └─────────┘
```

**Oscillation 방지 메커니즘**: cost가 동일한 셀이 여러 개일 때, heading 방향과의 **정렬도(alignment = cos(angle))** 가 높은 셀을 선택한다. 이렇게 하면 좌우로 왔다갔다하는 진동(oscillation)을 방지할 수 있다.

### 9.3 좌표 변환 헬퍼

```cpp
// (magnetic_planner.cpp:10-31)
// world → grid
bool MagneticPlanner::world_to_grid(
  double wx, double wy, ...)
{
  col = static_cast<int>((wx - origin_x) / resolution);
  row = static_cast<int>((wy - origin_y) / resolution);
  return (row >= 0 && row < rows && col >= 0 && col < cols);
}

// grid → world (셀 중심 좌표)
Point2D MagneticPlanner::grid_to_world(
  int row, int col, ...)
{
  return {
    origin_x + (col + 0.5) * resolution,   // 0.5 offset → 셀 정중앙
    origin_y + (row + 0.5) * resolution
  };
}
```

---

## 10. Stage 4: Postprocess — 경로 후처리

**목적**: Greedy 플래너가 생성한 raw path는 셀 단위로 지그재그하므로, **가지치기 → 평활화 → 리샘플링** 3단계를 거쳐 부드러운 최종 경로를 생성한다.

### 10.1 process() — 후처리 메인 진입점

```cpp
// (path_postprocessor.cpp:88-114)
PostprocessResult PathPostprocessor::process(
  const std::vector<Point2D> & raw_path,
  double prune_max_dev,
  int smooth_window,
  double resample_ds)
{
  PostprocessResult result;
  if (raw_path.size() < 2) return result;

  // [Step 1] 가지치기 — 불필요한 중간점 제거
  auto pruned = prune(raw_path, prune_max_dev);

  // [Step 2] 평활화 — 이동 평균 필터
  auto smoothed = smooth(pruned, smooth_window);

  // [Step 3] 리샘플링 — 일정 간격으로 보간
  result.path = resample_polyline(smoothed, resample_ds);

  if (result.path.size() < 2) return result;

  // [Step 4] yaw 계산 — 각 점의 접선 방향
  auto tangents = polyline_tangents(result.path);
  result.yaw.resize(result.path.size());
  for (size_t i = 0; i < tangents.size(); ++i) {
    result.yaw[i] = heading(tangents[i]);  // atan2(dy, dx)
  }

  result.valid = true;
  return result;
}
```

### 10.2 prune() — 가지치기 (Douglas-Peucker 변형)

직선으로 연결해도 횡편차가 `max_dev`(0.15m) 이내인 중간 점들을 제거한다:

```cpp
// (path_postprocessor.cpp:10-56)
std::vector<Point2D> PathPostprocessor::prune(
  const std::vector<Point2D> & pts, double max_dev)
{
  if (pts.size() <= 2) return pts;

  std::vector<Point2D> result;
  result.push_back(pts.front());  // 시작점은 항상 유지

  size_t i = 0;
  while (i < pts.size() - 1) {
    size_t best_j = i + 1;

    // 가장 먼 점부터 역순으로 탐색 → 최대 shortcut 시도
    for (size_t j = pts.size() - 1; j > i + 1; --j) {
      const Point2D & a = pts[i];
      const Point2D & b = pts[j];
      const double ab_len = dist(a, b);

      bool can_shortcut = true;
      if (ab_len < 1e-12) {
        can_shortcut = false;
      } else {
        const Point2D dir = {(b.x - a.x) / ab_len, (b.y - a.y) / ab_len};

        // i와 j 사이의 모든 중간점에 대해 횡편차 검사
        for (size_t k = i + 1; k < j; ++k) {
          const double dx = pts[k].x - a.x;
          const double dy = pts[k].y - a.y;
          // 외적으로 점-직선 거리(수직 거리) 계산
          const double perp = std::abs(dx * dir.y - dy * dir.x);

          if (perp > max_dev) {
            can_shortcut = false;
            break;
          }
        }
      }

      if (can_shortcut) {
        best_j = j;     // 가장 먼 shortcut 성공 지점
        break;
      }
    }

    result.push_back(pts[best_j]);
    i = best_j;
  }

  return result;
}
```

시각화:

```
  Before prune (raw_path):
  A ─ · ─ · ─ · ─ · ─ · ─ · ─ · ─ B     ← 셀 단위 지그재그

  After prune (max_dev = 0.15m):
  A ──────────────────────────────── B     ← 직선 shortcut
  (중간점들의 수직거리가 모두 0.15m 이내이면 제거)
```

### 10.3 smooth() — 이동 평균 평활화

```cpp
// (path_postprocessor.cpp:58-86)
std::vector<Point2D> PathPostprocessor::smooth(
  const std::vector<Point2D> & pts, int window)
{
  if (pts.size() <= 2 || window <= 1) return pts;

  const int half = window / 2;   // window=5 → half=2
  const int n = static_cast<int>(pts.size());

  std::vector<Point2D> result(pts.size());
  result.front() = pts.front();  // 시작점 고정
  result.back() = pts.back();    // 끝점 고정

  for (int i = 1; i < n - 1; ++i) {
    double sx = 0.0, sy = 0.0;
    int count = 0;

    const int lo = std::max(0, i - half);
    const int hi = std::min(n - 1, i + half);

    for (int j = lo; j <= hi; ++j) {
      sx += pts[j].x;
      sy += pts[j].y;
      ++count;
    }
    result[i] = {sx / count, sy / count};  // 윈도우 내 평균
  }

  return result;
}
```

> `window=5`일 때 각 점은 자신 포함 전후 2개씩, 총 5개 점의 **좌표 평균**으로 대체된다. 시작점과 끝점은 고정하여 경로 시작/종료 위치가 변하지 않도록 한다.

### 10.4 resample + yaw 계산

리샘플링은 `geometry.hpp`의 `resample_polyline()`을 사용하여 `resample_ds=0.10m` 간격으로 일정 배치한다.

이후 `polyline_tangents()`로 각 점의 접선 벡터를 구하고, `heading()` = `atan2(dy, dx)`로 yaw 각도를 계산한다.

전체 후처리 흐름:

```
raw_path (수십~수백 점, 불규칙 간격)
    │
    ▼ prune(max_dev=0.15m)
pruned (점 수 대폭 감소, 불규칙 간격)
    │
    ▼ smooth(window=5)
smoothed (곡선이 부드러워짐, 불규칙 간격)
    │
    ▼ resample_polyline(ds=0.10m)
resampled (일정 간격 0.10m, 컨트롤러 친화적)
    │
    ▼ polyline_tangents() + heading()
final (path + yaw, 각 점에 heading 포함)
```

---

## 11. Stage 5: Safety Check — 안전 검사 & 속도 결정

**목적**: 생성된 경로의 곡률을 분석하여 차량이 물리적으로 추종 가능한지 검사하고, 안전한 목표 속도를 결정한다.

### 11.1 compute_max_curvature() — 최대 곡률 계산

연속 3점으로 **원의 곡률(κ)** 을 계산한다:

```cpp
// (safety_checker.hpp:26-56)
inline double compute_max_curvature(const std::vector<Point2D> & path)
{
  if (path.size() < 3) return 0.0;

  double kappa_max = 0.0;

  for (size_t i = 0; i + 2 < path.size(); ++i) {
    const Point2D & a = path[i];
    const Point2D & b = path[i + 1];
    const Point2D & c = path[i + 2];

    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ac = dist(a, c);
    const double denom = ab * bc * ac;
    if (denom < 1e-12) continue;

    const Point2D ba = b - a;
    const Point2D cb = c - b;
    const double cross_val = std::abs(cross2(ba, cb));

    // 곡률 공식: κ = 2 * |외적| / (|AB| * |BC| * |AC|)
    const double kappa = 2.0 * cross_val / denom;

    if (kappa > kappa_max) kappa_max = kappa;
  }

  return kappa_max;
}
```

> **수학 배경**: 세 점 A, B, C를 지나는 원(circumscribed circle)의 반지름 R과 곡률 κ의 관계: `κ = 1/R = 4·Area / (|AB|·|BC|·|AC|)`. 여기서 삼각형 넓이 = `|cross(BA, CB)| / 2`이므로, `κ = 2·|cross| / (|AB|·|BC|·|AC|)`.

### 11.2 check() — 안전 판정 & 속도 결정

```cpp
// (safety_checker.hpp:58-96)
inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  // [Case 1] 유효한 경로가 없으면 → STOP
  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::STOP;
    result.reason = "no_valid_path";
    result.target_speed = 0.0;
    return result;
  }

  // 최대 곡률 계산
  result.max_curvature = compute_max_curvature(path.path);

  // 차량의 최소 회전 반경에서 허용 가능한 최대 곡률
  const double r_min = p.vehicle.r_min();     // wheelbase / tan(delta_max)
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  // [Case 2] 곡률이 차량 한계 초과 → INFEASIBLE
  if (result.max_curvature > kappa_limit) {
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  // [Case 3] 정상 → 곡률 기반 속도 결정
  double v_target = p.speed.v_max;  // 1.60 m/s

  if (result.max_curvature > 1e-6) {
    // 횡가속도 제약: a_lat = v² · κ ≤ a_lat_max
    // → v ≤ sqrt(a_lat_max / κ)
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);
  }

  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}
```

속도 결정 로직:

```
v_target = min(v_max, sqrt(a_lat_max / κ_max))
```

예시:
- 직선 (κ≈0): `v_target = 1.60 m/s` (최대 속도)
- 완만한 커브 (κ=0.5): `v_target = min(1.60, sqrt(2.0/0.5)) = min(1.60, 2.0) = 1.60`
- 급커브 (κ=2.0): `v_target = min(1.60, sqrt(2.0/2.0)) = min(1.60, 1.0) = 1.0`

차량 제원 기반 한계:
```
r_min = wheelbase / tan(delta_max) = 0.87 / tan(0.314) ≈ 0.87 / 0.3249 ≈ 2.68m
κ_limit = 1 / r_min ≈ 0.373 rad/m
```

---

## 12. Stage 6: Publish — 결과 퍼블리시

### 12.1 Core 퍼블리시

```cpp
// (mr_planner_node.cpp:132-143)
// 최종 경로 → /planning/path
auto path_msg = std::make_unique<nav_msgs::msg::Path>(
  to_path_msg(pp_result.path, frame_id, stamp));
pub_path_->publish(std::move(path_msg));

// 플래너 상태 → /planning/status
auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
status_msg->header.stamp = stamp;
status_msg->header.frame_id = frame_id;
status_msg->status = static_cast<uint8_t>(safety.state);
status_msg->reason = safety.reason;
pub_status_->publish(std::move(status_msg));
```

> `std::make_unique`로 생성하고 `std::move`로 전달 → intra-process 환경에서 **Zero-copy** 퍼블리시.

### 12.2 Debug 퍼블리시 (Lazy Publishing)

```cpp
// (mr_planner_node.cpp:146-168)
// 코스트맵 시각화 → /planning/debug/costmap (구독자가 있을 때만)
if (pub_dbg_costmap_->get_subscription_count() > 0 && costmap.valid) {
  auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  grid_msg->header.stamp = stamp;
  grid_msg->header.frame_id = frame_id;
  grid_msg->info.resolution = static_cast<float>(costmap.resolution);
  grid_msg->info.width = costmap.cols;
  grid_msg->info.height = costmap.rows;
  grid_msg->info.origin.position.x = costmap.origin_x;
  grid_msg->info.origin.position.y = costmap.origin_y;
  grid_msg->info.origin.orientation.w = 1.0;
  grid_msg->data.resize(costmap.rows * costmap.cols);
  for (size_t i = 0; i < costmap.data.size(); ++i) {
    grid_msg->data[i] = static_cast<int8_t>(
      std::clamp(costmap.data[i], 0.0, 100.0));  // double → int8_t 변환
  }
  pub_dbg_costmap_->publish(std::move(grid_msg));
}

// 후처리 전 경로 → /planning/debug/raw_path (구독자가 있을 때만)
if (pub_dbg_raw_path_->get_subscription_count() > 0) {
  pub_dbg_raw_path_->publish(std::make_unique<nav_msgs::msg::Path>(
    to_path_msg(raw_path, frame_id, stamp)));
}
```

> **Lazy Publishing**: `get_subscription_count() > 0`으로 구독자가 있을 때만 디버그 메시지를 생성/퍼블리시한다. RViz2를 열지 않은 상태에서는 **불필요한 OccupancyGrid 변환 비용을 절약**한다.

---

## 13. 파라미터 일람표

| 카테고리 | 파라미터 | 기본값 | 단위 | 설명 |
|---------|---------|--------|------|------|
| **costmap** | size_x | 10.0 | m | 그리드 전체 폭 (ego 중심 ±5m) |
| | size_y | 10.0 | m | 그리드 전체 높이 |
| | resolution | 0.05 | m/cell | 셀 크기 (200×200 = 40,000 cells) |
| | cone_cost_max | 100.0 | - | 콘 중심 최대 cost |
| | lane_cost_max | 50.0 | - | 차선 경계점 최대 cost |
| | cone_radius | 0.65 | m | 콘 클러스터 반지름 (flat zone) |
| | alpha | 2.0 | - | 감쇠율 α (1/r² 공식의 계수) |
| | cost_threshold | 2.0 | - | cutoff 임계값 |
| **planner** | search_radius | 0.30 | m | 전방 180° 탐색 반경 |
| | max_steps | 200 | - | 최대 경로 포인트 수 |
| | heading_init_x | 1.0 | - | 초기 heading X (+x = 전방) |
| | heading_init_y | 0.0 | - | 초기 heading Y |
| **vehicle** | width | 0.50 | m | 차폭 (T870) |
| | wheelbase | 0.87 | m | 축간 거리 |
| | delta_max | 0.314 | rad | 최대 조향각 (~18°) |
| **safety** | margin | 0.10 | m | 안전 마진 |
| **speed** | v_max | 1.60 | m/s | 최대 속도 |
| | a_lat_max | 2.0 | m/s² | 최대 횡가속도 |
| **postprocess** | resample_ds | 0.10 | m | 출력 경로 포인트 간격 |
| | smooth_window | 5 | - | 이동 평균 윈도우 크기 |
| | prune_max_dev | 0.15 | m | 가지치기 최대 횡편차 |
| **timeouts** | perception_ms | 300 | ms | 인식 데이터 타임아웃 |

---

## 14. ROS 2 토픽 인터페이스

### 14.1 Subscriptions (입력)

| 토픽 | 메시지 타입 | QoS | 설명 |
|------|-----------|-----|------|
| `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray` | Depth=1 | 차선 경계점 배열 |
| `/perception/cones` | `track_msgs/ConeArray` | Depth=1 | 콘(장애물) 위치 배열 |

### 14.2 Publications (출력)

| 토픽 | 메시지 타입 | QoS | 설명 |
|------|-----------|-----|------|
| `/planning/path` | `nav_msgs/Path` | Depth=1 | **최종 경로** (후처리 완료) |
| `/planning/status` | `track_msgs/PlannerStatus` | Depth=1 | **플래너 상태** (OK/STOP/INFEASIBLE/STALE) |
| `/planning/debug/costmap` | `nav_msgs/OccupancyGrid` | Depth=1 | 코스트맵 시각화 (Lazy) |
| `/planning/debug/raw_path` | `nav_msgs/Path` | Depth=1 | 후처리 전 경로 (Lazy) |

### 14.3 Frame ID

모든 출력 메시지의 `frame_id`는 `"base_link"` (ego 차량 좌표계).
