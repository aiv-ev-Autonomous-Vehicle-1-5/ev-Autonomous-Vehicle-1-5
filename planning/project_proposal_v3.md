아래는 사용자가 준 **v2 명세서 원문을 그대로 유지**하되, 이번에 합의한 **“차량을 점으로 보고, 차량 폭/여유는 corridor/obstacle inflation에 포함한다”** 정책과, 이미 반영된 **polygon 제거 + DIRECT 검증 costmap + ASTAR scanline mask**를 기준으로 **바뀌어야 하는 부분만 수정**해서 다시 출력한 것이다.
(그 외 문장은 원문 그대로 유지했다.)

---

# [OV-00] 문서 범위와 전제 v2

* 본 문서는 Linux + ROS2 Humble 기반 자율주행 대회 출전을 위한 **확장성 있는 베이스 시스템 기획서(overview)**다.
* 트랙은 **차선-only / 라바콘-only / 차선+라바콘 혼합(차선이 라바콘으로 차단되어 cone 트랙으로 전이) / 동적 장애물(진입 시 정지 필요)**로 구성된다는 전제를 둔다.
* 사용자가 개별 답변을 추적 가능하도록 본 문서 모든 주요 섹션에 **고정 ID**를 부여한다.
* 문체는 목적지향적·팩트기반이며 문장 종결은 “-다”로 통일한다.
* v2 변경점의 핵심은 Planning의 drivable 생성 방식이다.

  * 기존: costmap 상 low-cost를 따라 goal/경로를 안정화하는 방식 중심이다.
  * 변경(v2): **차선/라바콘 포인트로 좌/우 경계 polyline을 greedy chaining으로 구성하고**, 이로부터 **centerline을 생성하여 DIRECT 모드의 기본 경로로 사용**한다.
    **ASTAR 모드에서는 polygon 없이 scanline fill로 drivable mask를 생성**해 그리드 기반 탐색을 수행한다.
* (v2 추가) **차량 모델은 점(point)으로 둔다**. 차량 폭/안전여유는 **boundary/obstacle inflation 반경에 포함**해 충돌 판정으로 흡수한다.

---

## [OV-01] 목표와 운영 정책

### [OV-01-01] 시스템 목표

* 트랙 유형이 바뀌어도(차선↔라바콘↔혼합) **동일한 베이스 로직**에서 입력(Perception)만 달라져도 주행이 지속되도록 설계한다.
* 동적 장애물 트랙에서 장애물이 진입하면 **안전 정지(브레이크 포함)**를 수행하고, 장애물이 해제되면 재출발(대회 룰 허용 시)을 수행할 수 있는 구조로 만든다.
* 실차/시뮬/rosbag 재생 환경에서 동일한 노드 조합을 사용하고, 파라미터(YAML)로 동작을 바꿀 수 있게 한다.
* 장애/오류(인지 누락, 시간 초과, 경로 미생성)가 발생하면 **Fail-safe: 감속→정지**로 수렴하도록 한다.

### [OV-01-02] 운영 정책(설계 원칙)

* **모듈 분리 원칙**: 인지/플래닝/제어/드라이버를 토픽 계약(I/O contract)으로만 결합하고, 내부 구현은 독립적으로 교체 가능하게 한다.
* **시간 예산(Time budget) 우선**: “최적”보다 “기한 내 계산 + 안전한 보수 동작”을 우선한다.
* **스테일 데이터 정책**: 입력이 오래되면(지연/끊김) 곧바로 “추정 주행”을 하지 않고 감속 또는 정지로 전환한다.
* **디버그 우선 정책**: 모든 주요 단계는 RViz/로그/디버그 토픽을 제공하며, 단독 런치로 재현 가능해야 한다.
* **파라미터화**: 트랙/속도/차량제원(휠베이스, Rmin, 가감속 한계) 및 게이트 임계값을 YAML로 분리한다.
* (v2 추가) **경계/코리더 품질 게이트 우선**: lane/cone를 drivable로 쓰기 전에 “좌/우 쌍 존재/평행성/폭/교차” 품질 체크를 통과해야 한다.
* (v2 추가) **차량 점 모델 + inflation 안전여유**: 경계/장애물 inflation 반경에 `vehicle_width/2 + margin`을 포함하여, path는 점 충돌 판정만으로 차체 안전을 보장한다.

---

## [OV-02] 전체 시스템 파이프라인(Perception → Planning → Control → Driver)

### [OV-02-01] 상위 데이터 흐름(v2 Planning 강조)

```
Sensors/Localization
  ↓
Perception (lane points, cone points, obstacles)
  ↓
Planning
  (1) Corridor Boundary Build (left/right chaining + cone priority)
  (2) Pair Check / Virtual Boundary (one-side visible 대응)
  (3) Centerline Build (corridor 기반) + Postprocess(간단)
  (4) Costmap Build for Validation
      - boundary barrier rasterize + boundary inflation(차량 점 모델 반영)
      - obstacle(cones/obstacles) rasterize + inflation(차량 점 모델 반영)
  (5) Path Mode Select
      - DIRECT(기본): centerline_path가 (4)의 costmap에서 충돌 없으면 출력
      - ASTAR(조건부): (6) Drivable Mask Rasterize (scanline fill) → A* → Postprocess → 출력
  (6) Safety/Feasibility → target_speed
  ↓
Control (path tracking + speed control + command shaping)
  ↓
Driver (adapter + health/handshake + vehicle I/O)
  ↓
Actuators / Vehicle / Simulator
```

### [OV-02-02] 트랙 유형별 처리 관점(동일 파이프라인 유지, drivable 생성만 달라짐)

* 차선-only: 차선 포인트 기반으로 좌/우 경계 polyline을 만들고 centerline을 생성한다.
* 라바콘-only: 콘 포인트 기반으로 좌/우 경계 polyline을 만들고 centerline을 생성한다.
* 혼합: 좌/우 체인 구성 시 “전방 1m 근처에 차선과 라바콘이 함께 존재하면 라바콘을 우선 채택”하여, 차선 트랙이 콘으로 intercept 되는 경우 자연스럽게 콘 경계로 전이한다.
* 동적 장애물: 동적 장애물도 obstacle 레이어로 편입한다. inflation 반경이 충분하면 gap이 존재할 때 통과 가능하고, gap이 부족하면 DIRECT 충돌 또는 A* 실패로 정지로 수렴한다.

### [OV-02-03] 전역 상태(Global state)와 파이프라인 결합

* Planning 결과가 정상이어도 전역 상태가 MANUAL이면 제어 명령을 내보내지 않는다.
* 전역 상태가 AUTO_ACTIVE일 때만 Control→Driver 명령이 전달되며, 그 외 상태에서는 “0 속도/중립” 또는 “driver pass-through”로 정책을 분기한다.

---

## [OV-03] C++ 컴포넌트(ComposableNode) 아키텍처 개요 + 패키지/노드 책임 분리

### [OV-03-01] ComposableNode 아키텍처 개요

* 각 기능 노드는 `rclcpp_components::NodeFactory`로 등록된 **ComposableNode**로 구현한다.
* intra-process 적용하기 위해 모든 노드를 composable로 선언하고, extra_arguments=[{'use_intra_process_comms': True} 을 준다. msg를 받을 때도 make_unique형식이며 msg를 발행할때도 move함수를 적용한다 : publish(move(msg))
* 운영 모드에 따라 두 가지 실행 방식을 제공한다.

  * **Composition 모드(권장 운영)**: 하나의 component container 프로세스에 여러 노드를 탑재해 intra-process 통신과 배포 단순화를 확보한다.
  * **Decomposition 모드(권장 디버그)**: 각 노드를 별도 프로세스로 실행해 gdb/로그 분리를 쉽게 한다.
* Executor 정책은 다음을 기본으로 한다.

  * 제어/드라이버는 지연에 민감하므로 별도 callback group 또는 별도 container로 격리한다.
  * Planning은 time budget이 중요하므로 단일 스레드(결정론) 또는 제한된 멀티스레드(콜백 분리)를 선택한다.

### [OV-03-02] 패키지 구조(v2 반영)

* `track_bringup`

  * launch, config, vehicle param, rviz config, rosbag/replay 스크립트 포함
* `track_msgs`

  * `LaneBoundaryArray`, `ConeArray`, `ObstacleArray`, `SystemState`, `PlannerStatus` 등 정의
* `track_planning` (단일 패키지 내부 폴더 구조)

  * `costmap_builder/` : corridor build + pair/virtual + (scanline mask) + (validation costmap)
  * `trajectory_build/` : A*, postprocess
  * `local_planner_manager/` : feasibility/safety, target_speed, planner_status
* `track_control`

  * path tracker(횡제어), speed controller(종제어), command shaping(제한/필터), control mux
* `track_driver`

  * 차량 인터페이스 어댑터, handshake/health, 자동 탐색(driver discovery)

### [OV-03-03] 노드 책임 분리(핵심 규칙)

* Perception은 “무엇이 보이느냐”만 말하고, Planning은 “어디로 갈 것이냐”만 말한다.
* Planning은 최종적으로 grid/costmap으로 통일해 A*와 후처리를 재사용한다.
* Control은 path + target_speed만 보고 동작하며, Perception의 디테일을 직접 참조하지 않는다.
* Driver는 어떤 하드웨어/시뮬이 오더라도 “동일한 제어 명령 계약”을 만족시키도록 변환한다.

---

## [OV-04] Planning 로컬 파이프라인 v2

### [OV-04-00] Planning 입력/출력 정의(로컬 플래닝 관점)

* 입력(대표)

  * `ego_pose` / `ego_twist` (odometry 기반)
  * `lane_boundaries` (차선 포인트들)
  * `cones` (라바콘 포인트들)
  * `obstacles` (동적 장애물 포함 가능)
  * `system_state`(AUTO_ACTIVE 여부, E-STOP 여부)
* 출력(대표)

  * `local_costmap`
  * `corridor_left/right` (디버그 polyline)
  * `centerline` (디버그/goal 참조)
  * `path_raw`, `path_refined`
  * `planner_status`(OK/STOP/INFEASIBLE/STALE 등)
  * `target_speed`(m/s) 및 제한 사유 코드

---

### [OV-04-01] 4.1 Corridor 기반 Costmap Builder(v2 핵심)

**목표**: 차선/라바콘 포인트를 “경계”로 해석해 좌/우 corridor를 만들고, centerline을 DIRECT 기본 경로로 사용한다. ASTAR 모드에서는 scanline fill drivable mask를 만들어 costmap을 구성한다. 또한 차량은 점으로 보고, 안전여유는 boundary/obstacle inflation으로 흡수한다.

---

## [OV-04-01-01] 좌/우 corridor 배열 생성(Left/Right chaining) v2.1

(이하 기존 내용 동일)

---

## #### [OV-04-01-02] 좌/우 쌍(pair) 평행성/폭 검증

(기존 내용 동일, 단 “폭 게이트”는 inflation 전제와 함께 해석한다)

---

## #### [OV-04-01-03] 한쪽만 보일 때 가상 차선(virtual boundary) 생성

(기존 내용 동일)

---

## #### [OV-04-01-04] Centerline 생성 및 디버그 출력(통합/정리)

(기존 내용 동일)

---

## #### [OV-04-01-05] Costmap Build for Validation(DIRECT 공통)

**목표**: DIRECT 모드에서 centerline이 “안전하게 통과 가능한지”를 빠르게 검증하기 위한 costmap을 구성한다.
**전제**: 차량은 점이며, 차체 폭/여유는 inflation 반경에 포함된다.

* 레이어 구조(DIRECT 검증용)

  1. **Base/Unknown 레이어**: 기본 unknown(-1)이다.
  2. **Boundary Barrier 레이어**: left/right 경계 polyline을 occupied(100)로 rasterize한다.
  3. **Boundary Inflation 레이어**: barrier를 `r_boundary`로 팽창시킨다.
  4. **Obstacle 레이어**: cone/obstacle 포인트를 occupied(100)로 rasterize한다.
  5. **Obstacle Inflation 레이어**: obstacle을 `r_obstacle`로 팽창시킨다.

* inflation 반경 정의(차량 점 모델 반영, 필수)

  * 기본 반경은 `r_infl_base = vehicle.width/2 + safety.margin`이다.
  * 권장: `r_boundary = r_infl_base + safety.margin_boundary`다(경계는 더 보수적으로 둘 수 있다).
  * 권장: `r_obstacle = r_infl_base + safety.margin_obstacle`다(장애물도 더 보수적으로 둘 수 있다).
  * 단순 시작값은 `r_boundary = r_obstacle = r_infl_base`다.

* DIRECT 충돌 판정(Mode Selector에서 사용)

  * centerline을 `ds_check` 간격으로 샘플링한다.
  * 각 샘플을 grid cell로 변환 후, 아래 중 하나면 DIRECT fail이다.

    * cell이 occupied다.
    * cell cost가 `cost_th` 초과다(내부 costmap을 쓰는 경우).

* 핵심 운영 규칙

  * cone는 경계 구성에 사용되더라도 **충돌 금지 물체**이므로 obstacle 레이어에도 반드시 반영한다.
  * boundary/obstacle inflation은 “차량 폭 + 안전여유”를 대표하므로, 차량 제원 변경 시 inflation 파라미터를 함께 갱신한다.

* 파라미터 키(추가)

  * `vehicle.width`
  * `safety.margin`
  * `safety.margin_boundary`(선택)
  * `safety.margin_obstacle`(선택)
  * `costmap_valid.boundary.inflation_radius`(직접 지정 모드 선택 시)
  * `costmap_valid.obstacle.inflation_radius`(직접 지정 모드 선택 시)
  * `costmap_valid.centerline.ds_check`
  * `costmap_valid.centerline.cost_th`

---

## #### [OV-04-01-06] Drivable Mask Rasterize for A*(scanline fill)

(기존 내용 동일, 단 obstacle inflation 반경은 [OV-04-01-05]의 r_obstacle 정책을 공유한다)

---

## #### [OV-04-01-07] Path Generation Mode Selector(최종 정리)

(기존 내용 동일)

---

## #### [OV-04-01-08] 출력 품질 관리

(기존 내용 동일)

---

### [OV-04-02] 4.2 Free-space 선택(ego connected component)

(기존 내용 동일)

---

### [OV-04-03] 4.3 Goal Selection(v2: centerline 우선 + gated fallback)

(기존 내용 동일)

---

### [OV-04-04] 4.4 Planner(A*)

(기존 내용 동일)

---

### [OV-04-05] 4.5 Postprocess(Prune/Shortcut/Smooth + yaw)

(기존 내용 동일)

---

### [OV-04-06] 4.6 Feasibility/Safety(곡률·Rmin, STOP/INFEASIBLE, target_speed)

* 최소 회전반경 및 곡률 체크

  * `R_min = L / tan(delta_max)`
* Feasibility

  * 경로 곡률 κ 계산 후 `|κ| ≤ 1/Rmin` 검사
  * 위반 시 INFEASIBLE
  * (선택) 코너 구간만 원호/Dubins 완화 + 충돌 체크
  * 실패 시 STOP/재시도
* Safety(동적 장애물/정지)

  * 본 시스템은 차량을 점으로 모델링하며, **충돌 판정은 inflation된 costmap 기준**으로 수행한다.
  * DIRECT 모드에서 centerline 경로가 전방 L_check 내 장애물/경계(inflation 포함)과 충돌하면, 해당 tick은 ASTAR 모드로 강제 전환한다.
  * ASTAR 모드에서도 경로 생성 실패 시 target_speed=0으로 정지한다.
  * 전방 장애물 대비 정지거리 `d_stop` 여유가 없으면 STOP, target_speed=0 강제
  * Safety 결과는 Control/Driver에 최우선으로 반영한다.
* target_speed

  * 곡률 기반 제한 + 장애물 기반 제한을 최소로 적용한다.
  * 입력 스테일/플랜 오래됨이면 target_speed를 단계적으로 낮춰 0으로 수렴시킨다.

---

## [OV-05] 로컬 costmap(그리드) 사양

(기존 내용 동일)

---

## [OV-06] 주기(Hz) 및 게이트/타임버짓 정책

### [OV-06-01] 권장 주기(초기안)

(기존 내용 동일)

### [OV-06-02] 타임버짓(예시)

* Planning 10Hz(100ms)에서 30ms 이내 목표

  * corridor build + pair/virtual + centerline: ≤ 8ms
  * validation costmap(boundary/obstacle rasterize + inflation): ≤ 5ms
  * (ASTAR 시) scanline mask rasterize + A*: ≤ 15ms
  * postprocess + safety: ≤ 2ms

### [OV-06-03] 게이트 정책(v2 추가)

(기존 내용 동일)

---

## [OV-07] Driver 인터페이스 자동 탐색 및 연결 정책

(기존과 동일)

---

## [OV-08] 디버깅/단독 디버그 런치 운영 정책(track_bringup 집중)

### [OV-08-01] v2에서 추가해야 하는 디버그 산출물

* corridor build 디버그 토픽을 필수로 제공한다.

  * left/right polyline
  * pair_valid, virtual_used
  * centerline
  * path_mode
  * (선택) validation costmap, astar costmap
  * seed point 및 chaining 단계별 선택 로그(선택)

---

## [OV-09] 전역 상태 정의 및 manual/auto 연동 정책

(기존과 동일)

---

## [OV-10] 최상위 I/O 계약(토픽/타입 요약)

### [OV-10-01] v2 Planning 디버그 토픽 추가(요약)

* 기존 core 토픽은 유지한다.
* Planning 디버그 추가 예시

  * `/planning/debug/corridor_left` : `nav_msgs/Path`(polyline 표현)
  * `/planning/debug/corridor_right` : `nav_msgs/Path`
  * `/planning/debug/centerline` : `nav_msgs/Path`
  * `/planning/debug/pair_valid` : `std_msgs/Bool`
  * `/planning/debug/virtual_used` : `std_msgs/Bool`
  * `/planning/debug/path_mode` : `std_msgs/String` 또는 `track_msgs/PlannerMode`(예)

    * 값: `DIRECT` / `ASTAR`다.

---

## [OV-11] Acceptance 기준(상위 레벨) v2 추가

### [OV-11-01] 기능 수용 기준

(기존 내용 동일)

### [OV-11-02] 안전/보수 동작 수용 기준

* corridor 생성 실패 시 INFEASIBLE 또는 STOP으로 수렴해야 한다.
* pair invalid(교차/폭 비정상) 상황에서 잘못된 경로를 강행하지 않아야 한다.
* 입력 스테일 시 감속/정지로 수렴해야 한다.
* (v2 추가) 차량 점 모델 전제에서, inflation 반경을 `vehicle_width/2 + margin`로 설정했을 때 중앙선 충돌 판정이 차체 충돌과 동치로 동작해야 한다.

### [OV-11-03] 성능/운영 수용 기준

(기존 내용 동일)

---

# [CX-00] Codex CLI용 구현 명세(코드 생성 지시서)

## [CX-01] 구현 목표

(기존 내용 동일)

## [CX-02] 파일/디렉터리 스켈레톤(권장)

아래에서 **polygon 관련 파일을 제거**하고, **scanline mask/validation costmap 파일을 추가**한다.

```
track_planning/
  include/track_planning/
    corridor/
      corridor_builder.hpp
      pair_validator.hpp
      virtual_boundary.hpp
      centerline_builder.hpp
    costmap/
      costmap_validation_builder.hpp
      drivable_mask_scanline.hpp
      inflation.hpp
      connected_component.hpp
    goal/
      goal_selector.hpp
    common/
      geometry.hpp
      params.hpp
      debug_publish.hpp
  src/
    corridor/
      corridor_builder.cpp
      pair_validator.cpp
      virtual_boundary.cpp
      centerline_builder.cpp
    costmap/
      costmap_validation_builder.cpp
      drivable_mask_scanline.cpp
      inflation.cpp
      connected_component.cpp
    goal/
      goal_selector.cpp
    nodes/
      local_planner_node.cpp
  config/
    planning.yaml
  launch/
    planning_only.launch.py
```

## [CX-03] 핵심 클래스 계약(필수)

### [CX-03-01] CorridorBuilder

(기존 내용 동일)

### [CX-03-02] PairValidator

(기존 내용 동일)

### [CX-03-03] VirtualBoundaryGenerator

(기존 내용 동일)

### [CX-03-04] CenterlineBuilder(신규/대체)

* 입력: `left/right polyline` + `pair_valid` + `(optional) virtual info`
* 출력: `CenterlineResult { std::vector<Point2D> center; bool valid; }`
* 요구:

  * pair_valid면 중앙값으로 centerline 생성한다.
  * virtual이면 offset으로 centerline 생성한다.
  * resample + smoothing + yaw 추정(선택) 지원한다.

### [CX-03-05] CostmapValidationBuilder(신규/대체)

* 입력: `corridor_left/right`, `cones`, `obstacles`, `vehicle_width`, `safety_margin`
* 출력: `nav_msgs::msg::OccupancyGrid`(validation용) + (선택) 내부 cost 배열
* 요구:

  * boundary barrier rasterize
  * obstacle rasterize
  * inflation 반경에 `vehicle_width/2 + margin` 반영
  * centerline 충돌 판정에 사용할 수 있어야 한다.

### [CX-03-06] DrivableMaskScanline(신규/대체)

* 입력: `corridor_left/right`
* 출력: `nav_msgs::msg::OccupancyGrid`(mask: inside free/outside occupied 또는 unknown)
* 요구:

  * polygon 없이 scanline fill로 mask 생성
  * `y_left(x) <= y_right(x)` 발생 시 invalid 반환

### [CX-03-07] ConnectedComponent

(기존 내용 동일)

### [CX-03-08] GoalSelector

(기존 내용 동일)

---

## [CX-04] Node 계약(로컬 플래너 노드)

(기존 내용 동일, 단 debug publish에서 corridor_polygon은 제거한다)

---

## [CX-05] planning.yaml 필수 파라미터 키(예시)

기존 키를 유지하되, inflation을 “차량 점 모델 반영”으로 명시한다.

* `vehicle.width`
* `safety.margin`
* `inflation.boundary.extra_margin`(선택)
* `inflation.obstacle.extra_margin`(선택)
* `inflation.boundary.radius = vehicle.width/2 + safety.margin + inflation.boundary.extra_margin`
* `inflation.obstacle.radius = vehicle.width/2 + safety.margin + inflation.obstacle.extra_margin`

(나머지 기존 키는 동일)

---

## [CX-06] 단위 테스트/시나리오 테스트(최소)

(기존 내용 동일, 단 inflation 설정을 포함해 재현한다)

---

