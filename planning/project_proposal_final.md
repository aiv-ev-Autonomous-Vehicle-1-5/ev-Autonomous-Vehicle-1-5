# ROS2 Humble 기반 자율주행 대회 베이스 시스템 최종 명세서

(Planning v2: Corridor chaining + DIRECT/ASTAR 듀얼 모드 + 차량 점 모델/Inflation, Codex CLI 구현 지시 포함)

---

## [OV-00] 문서 범위와 전제 v2

* 본 문서는 Linux + ROS2 Humble 기반 자율주행 대회 출전을 위한 **확장성 있는 베이스 시스템 기획서(overview)**다.
* 트랙은 **차선-only / 라바콘-only / 차선+라바콘 혼합(차선이 라바콘으로 차단되어 cone 트랙으로 전이) / 동적 장애물 트랙**으로 구성된다는 전제를 둔다. 
* 사용자가 개별 답변을 추적 가능하도록 본 문서 모든 주요 섹션에 **고정 ID**를 부여한다. 
* 문체는 목적지향적·팩트기반이며 문장 종결은 “-다”로 통일한다. 
* v2 변경점의 핵심은 Planning의 “주행 가능 영역(drivable) 생성 방식”과 “DIRECT/ASTAR 듀얼 경로 생성”이다.

  * 기존: costmap 상 저비용 경로를 따라 goal/경로를 안정화하는 방식 중심이다.
  * 변경(v2): **lane/cone 포인트로 좌/우 경계 polyline을 greedy chaining으로 구성**하고, 그로부터 centerline을 만든 뒤, **DIRECT 기본 경로**로 사용한다. DIRECT 경로가 **inflation된 validation costmap에서 충돌**하면 **ASTAR 모드로 전환**해 scanline fill로 drivable mask를 만들고 A*를 수행한다.
* 동적 장애물은 “라바콘과 동일 규격의 물체가 뭉텅이로 주행 경로에 난입했다가 사라지는 상황”을 포함한다고 가정한다. 시스템은 **통과 가능한 gap이 없으면 경로 생성 실패로 정지**하고, gap이 있으면 DIRECT/ASTAR 판단에 따라 회피 또는 통과를 수행한다.
* 차량은 **점(point) 모델**로 취급하고, 차체 폭/여유는 **inflation 반경**으로 반영한다. 충돌 판정은 **inflation된 costmap 기준**으로 수행한다. 

---

## [OV-01] 목표와 운영 정책

### [OV-01-01] 시스템 목표

* 트랙 유형이 바뀌어도(차선↔라바콘↔혼합) **동일한 베이스 로직**에서 입력(Perception)만 달라져도 주행이 지속되도록 설계한다.
* 동적 장애물이 경로를 충분히 차단하면 **안전 정지(브레이크 포함)**를 수행하고, 통과 가능하면 **회피 또는 통과**가 가능하도록 만든다.
* 실차/시뮬/rosbag 재생 환경에서 동일한 노드 조합을 사용하고, 파라미터(YAML)로 동작을 바꿀 수 있게 한다.
* 장애/오류(인지 누락, 시간 초과, 경로 미생성)가 발생하면 **Fail-safe: 감속→정지**로 수렴하도록 한다.

### [OV-01-02] 운영 정책(설계 원칙)

* **모듈 분리 원칙**: 인지/플래닝/제어/드라이버를 토픽 계약(I/O contract)으로만 결합하고, 내부 구현은 독립적으로 교체 가능하게 한다.
* **시간 예산(Time budget) 우선**: “최적”보다 “기한 내 계산 + 안전한 보수 동작”을 우선한다.
* **스테일 데이터 정책**: 입력이 오래되면(지연/끊김) 추정 주행을 하지 않고 감속 또는 정지로 전환한다.
* **디버그 우선 정책**: 모든 주요 단계는 RViz/로그/디버그 토픽을 제공하며, 단독 런치로 재현 가능해야 한다.
* **파라미터화**: 트랙/속도/차량제원 및 게이트 임계값을 YAML로 분리한다.
* (v2) **경계/코리더 품질 게이트 우선**: lane/cone를 drivable로 쓰기 전에 “좌/우 쌍 존재/평행성/폭/교차” 품질 체크를 통과해야 한다.
* (v2) **차량 점 모델 + inflation 원칙**: 차체 폭과 안전 여유는 grid inflation 반경으로 반영하고, path 충돌 판정은 inflation된 costmap으로만 수행한다. 

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
  (4) Costmap Build for Validation (DIRECT 공통)
      - boundary barrier rasterize + boundary inflation(차량 폭 반영)
      - obstacle(cones/obstacles) rasterize + obstacle inflation(차량 폭 반영)
  (5) Path Mode Select
      - DIRECT(기본): centerline_path가 (4)에서 충돌 없으면 출력
      - ASTAR(조건부): (6) Drivable Mask Rasterize (scanline fill) → A* → Postprocess → 출력
  (6) Safety/Feasibility → target_speed
  ↓
Control (path tracking + speed control + command shaping)
  ↓
Driver (adapter + health/handshake + vehicle I/O)
  ↓
Actuators / Vehicle / Simulator
```

### [OV-02-02] 트랙 유형별 처리 관점(동일 파이프라인 유지)

* 차선-only: 차선 포인트 기반으로 좌/우 경계 polyline을 만들고 centerline을 생성한다.
* 라바콘-only: 콘 포인트 기반으로 좌/우 경계 polyline을 만들고 centerline을 생성한다.
* 혼합: chaining 단계에서 **전방 탐색 범위 내 lane+cone 공존 시 cone 우선** 규칙으로 차선이 콘에 의해 intercept 되는 경우 콘 경계로 자연스럽게 전이한다.
* 동적 장애물: cones/obstacles로 들어온 물체를 costmap obstacle로 반영하고, 통과 가능하면 회피/통과하며 통과 불가하면 경로 실패로 정지한다.

### [OV-02-03] 전역 상태(Global state)와 파이프라인 결합

* Planning 결과가 정상이어도 전역 상태가 MANUAL이면 제어 명령을 내보내지 않는다.
* 전역 상태가 AUTO_ACTIVE일 때만 Control→Driver 명령이 전달된다.
* 그 외 상태에서는 “0 속도/중립” 또는 “driver pass-through”로 정책을 분기한다.

---

## [OV-03] C++ 컴포넌트(ComposableNode) 아키텍처 + 패키지/노드 책임 분리

### [OV-03-01] ComposableNode 아키텍처 개요

* 각 기능 노드는 `rclcpp_components::NodeFactory`로 등록된 **ComposableNode**로 구현한다.
* intra-process 적용을 위해 모든 노드를 composable로 선언하고 `extra_arguments=[{'use_intra_process_comms': True}]`를 준다.
* 메시지 수신은 가능하면 unique pointer 기반으로 받고, 발행은 `publish(std::move(msg))`로 복사 비용을 줄인다.
* 운영 모드에 따라 두 가지 실행 방식을 제공한다.

  * **Composition 모드(권장 운영)**: 하나의 component container 프로세스에 여러 노드를 탑재해 intra-process 통신과 배포 단순화를 확보한다.
  * **Decomposition 모드(권장 디버그)**: 각 노드를 별도 프로세스로 실행해 gdb/로그 분리를 쉽게 한다.
* Executor 정책은 다음을 기본으로 한다.

  * 제어/드라이버는 지연에 민감하므로 별도 callback group 또는 별도 container로 격리한다.
  * Planning은 time budget이 중요하므로 단일 스레드(결정론) 또는 제한된 멀티스레드(콜백 분리)를 선택한다.

### [OV-03-02] 패키지 구조(v2)

* `track_bringup`

  * launch, config, vehicle param, rviz config, rosbag/replay 스크립트 포함이다.
* `track_msgs`

  * `LaneBoundaryArray`, `ConeArray`, `ObstacleArray`, `SystemState`, `PlannerStatus`, (선택) `PlannerMode` 등을 정의한다.
* `track_planning` (ROS2 패키지)

  * `corridor/`: boundary chaining, pair/virtual, centerline
  * `costmap/`: validation costmap, scanline mask, inflation, connected component
  * `goal/`: goal selector
  * `trajectory/`: A*, postprocess
  * `manager/`: feasibility/safety, mode selector, target_speed
* `track_control`

  * path tracker(횡제어), speed controller(종제어), command shaping(제한/필터), control mux를 포함한다.
* `track_driver`

  * 차량 인터페이스 어댑터, handshake/health, 자동 탐색(driver discovery)을 포함한다.

### [OV-03-03] 노드 책임 분리(핵심 규칙)

* Perception은 “무엇이 보이느냐”만 말하고, Planning은 “어디로 갈 것이냐”만 말한다.
* Planning은 **DIRECT(센터라인)** 과 **ASTAR(그리드)** 를 모두 지원하되, 충돌 판정은 costmap으로 통일한다.
* Control은 `path + target_speed`만 보고 동작하며, Perception 디테일을 직접 참조하지 않는다.
* Driver는 어떤 하드웨어/시뮬이 오더라도 “동일한 제어 명령 계약”을 만족시키도록 변환한다.

---

## [OV-04] Planning 로컬 파이프라인 v2

### [OV-04-00] Planning 입력/출력 정의(로컬 플래닝 관점)

* 입력(대표)

  * `ego_pose` / `ego_twist` (odometry 기반)이다.
  * `lane_boundaries` (좌/우 차선 포인트들)이다.
  * `cones` (라바콘 포인트들)이다.
  * `obstacles` (동적 장애물 포함 가능)이다.
  * `system_state`(AUTO_ACTIVE 여부, E-STOP 여부)다.
* 출력(대표)

  * `local_costmap_valid`(DIRECT 검증용)이다.
  * `local_costmap_astar`(ASTAR용, 선택)이다.
  * `corridor_left/right`(디버그 polyline)다.
  * `centerline`(디버그/Direct 경로)다.
  * `path_raw`, `path_refined`다.
  * `planner_status`(OK/STOP/INFEASIBLE/STALE 등)다.
  * `target_speed`(m/s) 및 제한 사유 코드다.
  * (디버그) `path_mode`(DIRECT/ASTAR)다.

---

## [OV-04-01] Corridor 기반 Path/Costmap Builder(v2 핵심)

**목표**: lane/cone 포인트로 corridor 경계를 만들고 centerline을 생성해 DIRECT 경로로 사용한다. DIRECT가 실패하면 scanline fill 기반 drivable mask를 만들어 ASTAR로 폴백한다.

---

### [OV-04-01-01] 좌/우 corridor 배열 생성(Left/Right chaining) v2.1

#### [OV-04-01-01-00] 입력 데이터 전제(좌/우 분리)

* Planning은 아래 중 하나를 만족하는 입력을 요구한다.

  1. Perception이 이미 `left/right`로 분리된 lane boundary를 제공한다.
  2. cone 포인트는 ego 로컬 프레임에서 `y>0`이면 left 후보, `y<0`이면 right 후보로 1차 분리한다.
* 동적 장애물이 cone로 인지되는 경우를 고려해, boundary 후보에서 “중앙부(|y|가 작은 영역)”를 배제하는 **side filter**를 둘 수 있다.

  * 예: `abs(y) >= y_side_min_abs`만 boundary 후보로 사용한다.
* 위 정책은 “중앙에 난입한 장애물”이 경계 체인에 끼어드는 것을 줄이기 위한 보수 장치다.

#### [OV-04-01-01-A] Reference(중앙차선 기준) 정의(채택: CH-12)

* chaining의 전방성/가까움 기준은 **중앙차선(centerline)의 끝점과 접선**을 기준으로 정의한다.
* reference point는 `c_end`(중앙차선 끝점)이다.
* reference tangent는 `t_end`(중앙차선 진행 방향 단위벡터)다.
* `t_end`는 마지막 2점이 아니라 **마지막 N점(권장 5~10점) 방향 회귀**로 계산한다.

  * 구현 예: 마지막 N점 평균 방향 벡터를 정규화한다.
  * 또는 PCA/선형회귀로 1차 주성분 방향을 구해 정규화한다.
* centerline이 아직 없거나 신뢰도가 낮으면 다음 순서로 fallback한다.

  1. 직전 프레임 `centerline_prev`의 `c_end/t_end`다.
  2. 직전 프레임 경계선(좌/우) 중 유효한 쪽의 마지막 접선이다.
  3. ego heading(+x)이다.

#### [OV-04-01-01-B] Cold Start(초기 프레임) 기준 c_end/t_end 결정 규칙

* cold start는 아래 중 하나를 만족하는 상태다.

  1. centerline이 아직 생성되지 않았다.
  2. centerline은 존재하나 길이가 `min_centerline_points` 미만이다.
  3. `pair_valid=false`이고 `virtual_used=false`로 corridor 품질이 매우 낮다.
* cold start에서 reference는 다음 순서로 결정한다.

  1. 이전 프레임 centerline_prev가 존재하면 `c_end = centerline_prev.back()`이고 `t_end = regress_tangent(centerline_prev, N_reg)`다.
  2. 이전 centerline이 없고 이번 프레임에 좌/우 경계 중 하나라도 seed가 잡혔으면 `c_end = ego_position`, `t_end = normalize(seed - ego_position)`이다.

     * 단, `dot(t_end, ego_heading) < cos(theta_heading_gate)`이면 3)으로 간다.
  3. 위 두 조건이 모두 불가하면 `c_end = ego_position`, `t_end = ego_heading(+x)`다.
* 파라미터 키는 다음과 같다.

  * `corridor.cold_start.min_centerline_points`다.
  * `corridor.cold_start.theta_heading_gate_deg`다.

#### [OV-04-01-01-C] Seed(초기 포인트) 선택 규칙(요구사항 반영)

* 각 side(left/right)별 후보(차선+콘) 중 **x가 가장 작은 포인트**를 seed로 선택한다.
* 뒤쪽(-x) noise를 피하기 위해 `x >= x_seed_min`을 만족하는 점만 seed 후보로 둔다.
* 파라미터 키는 `corridor.seed.x_seed_min`이다.

#### [OV-04-01-01-D] Candidate Pool 구성(채택: CH-11)

* 매 스텝 k에서 직전 경계 포인트 `p_k`가 주어졌을 때 탐색 범위 안에서 후보를 모은다.
* 후보 집합을 source 별로 나눈다.

  * `C_cone`: cone 후보들이다.
  * `C_lane`: lane 후보들이다.
* cone 우선 규칙은 점수화 전에 적용한다.

  * 필터 통과 cone 후보가 1개라도 있으면 후보 풀은 `C_cone`만 사용한다.
  * cone 후보가 없으면 후보 풀은 `C_lane`만 사용한다.
* 목적은 “cone이 있으면 lane을 아예 보지 않는다”를 명확히 구현하는 데 있다.

#### [OV-04-01-01-E] 후보 필터링(채택: CH-10, 1단계)

* 후보는 단순 거리 원만 쓰지 않고 중앙차선 진행좌표 기반으로 필터링한다.
* 각 후보 점 `p`에 대해 다음을 계산한다.

  * 전방 진행량 `s = dot(p - c_end, t_end)`다.
  * 접선에 대한 횡거리 `d = abs(cross2(t_end, p - c_end))`다.
* 필터 조건(파라미터화)은 다음과 같다.

  * 전방성: `s >= s_min`이다.
  * 전방 탐색 상한: `s <= s_max`다.
  * 횡방향 제한: `d <= d_max`다.
  * (선택) 원형 거리 제한: `dist(p, p_k) <= r_search`다.
* 필터 목적은 “뒤/옆 라인 점프”를 제거하고 진행 방향에 정렬된 후보만 남기는 데 있다.

#### [OV-04-01-01-F] 후보 점수화 및 선택(채택: CH-10-02, 2단계)

* 필터를 통과한 후보 풀에서 최종 1개를 선택한다.
* 기본은 점수 최대(score max) 선택이다.
* 점수는 “멀수록(전방 진행량 큼) + 중앙차선 접선에 가까울수록(횡거리 작음)”을 강하게 반영한다.
* 점수 구성(예시)은 다음과 같다.

  * `score(p) = + w_s*norm_s(s) - w_d*norm_d(d) - w_a*norm_a(Δθ) - w_p*norm_p(pred_err)`다.
  * `Δθ = angle_diff(t_k, p - p_k)`다.
  * `p_pred = p_k + step_pred * t_k`다.
  * `pred_err = dist(p, p_pred)`다.
  * `t_k`는 직전 두 경계 포인트로 만든 접선이며, 초기에는 `t_end` 또는 ego heading을 사용한다.
* 후보가 여러 개인 tie-break는 아래 비용 최소를 사용한다.

  * `J = w_d*dist(p_pred,p) + w_a*angle_diff(t_k, p - p_k)`다.
* 선택 규칙은 `score`가 가장 큰 후보를 `p_{k+1}`로 채택하는 것이다.

#### [OV-04-01-01-G] “접선에 가까운 후보 우선” Top-K 최적화(선택)

* 후보 수가 많을 때 연산을 줄이기 위해 아래를 적용할 수 있다.

  * 필터 통과 후보를 `d` 오름차순으로 정렬한다.
  * 상위 `K_top`개만 남긴다.
  * 그 `K_top`개에 대해서만 점수 계산을 수행한다.
* 이 방법은 “접선에 가까운 점 우선”을 보장하면서 score 기반 안정성을 유지한다.

#### [OV-04-01-01-H] 종료 조건

* 다음 조건 중 하나를 만족하면 체인을 종료한다.

  * 다음 후보가 없을 때다.
  * 포인트 수가 `max_points_side`를 초과할 때다.
  * x가 ROI 끝(`x_max`)에 도달했을 때다.

#### [OV-04-01-01-I] 파라미터 키(정리)

* Seed: `corridor.seed.x_seed_min`이다.
* Filter: `corridor.filter.s_min`, `corridor.filter.s_max`, `corridor.filter.d_max`, `corridor.filter.r_search(선택)`다.
* Scoring: `corridor.score.w_s`, `w_d`, `w_a`, `w_p`, `theta_max`, `step_pred`다.
* Top-K: `corridor.topk.enable`, `corridor.topk.K_top`다.
* Reference tangent: `corridor.ref_tangent.N_reg`다.
* Side filter(선택): `corridor.side_filter.y_side_min_abs`다.

---

### [OV-04-01-02] 좌/우 쌍(pair) 평행성/폭 검증

**목표**: 좌/우 경계가 서로 다른 라인으로 분기하거나 교차하는 경우를 감지해 드라이빙 실패를 예방한다.

* 입력: `corridor_left[]`, `corridor_right[]`다.
* 전처리

  * 두 polyline을 동일 간격(`resample_ds`)으로 resample한다.
  * 각 샘플에서 tangent를 구한다.
* 평행성 검사

  * 각 샘플 i에 대해 `Δθ_i = angle(tL_i, tR_i)`를 구한다.
  * 조건 예: `mean(Δθ_i) < theta_mean_th` AND `max(Δθ_i) < theta_max_th`다.
* 폭 검사

  * 각 샘플 i에서 폭 `w_i`를 계산한다.
  * 조건 예: `w_min < median(w_i) < w_max` AND `std(w_i) < w_std_th`다.
* 교차/뒤집힘 검사

  * 폭이 음수로 뒤집히거나 polyline 교차가 감지되면 `pair_valid=false`다.
* 결과

  * `pair_valid=true`이면 실측 좌/우 기반 centerline을 생성한다.
  * `pair_valid=false`이면 [OV-04-01-03] 가상 경계 생성으로 넘어간다.

---

### [OV-04-01-03] 한쪽만 보일 때 가상 경계(virtual boundary) 생성

**목표**: 코너 등에서 한쪽 경계만 인식되는 경우에도 centerline을 생성해 주행을 유지한다.

* 입력 케이스

  * left만 존재, right 없음이다.
  * right만 존재, left 없음이다.
  * 또는 `pair_valid=false`지만 한쪽이 상대적으로 품질이 더 좋다고 판단되는 경우다.
* 폭 추정값 `w_hat` 정책

  1. 직전 프레임 `pair_valid=true`였을 때의 `median(w_i)`를 우선 사용한다.
  2. 직전 값이 없으면 `default_track_width`를 사용한다.
  3. `w_hat`는 EMA로 저역통과 갱신한다.
* 가상 경계 생성 수식

  * 보이는 경계가 `P_vis`일 때, 각 점에서 접선 `t` 및 법선 `n=rot90(t)`를 구한다.
  * 가상 점은 `p_virtual = p_vis + sgn * w_hat * n`이다.
  * `sgn`은 좌/우 side에 따라 결정한다.
* 가상 경계 품질 게이트

  * corridor 최소 폭이 `min_corridor_width` 미만이면 실패다.
  * 가상 경계가 ROI 밖으로 크게 벗어나면 실패다.
  * (선택) obstacle/inflation과 충돌이 과도하면 실패다.
* 결과

  * 성공 시 `virtual_used=true`로 두고 centerline을 생성한다.
  * 실패 시 `virtual_used=false`로 두고 Mode Selector에서 ASTAR 또는 STOP/INFEASIBLE로 처리한다.

---

### [OV-04-01-04] Centerline 생성 및 디버그 출력

**목표**: 좌/우 경계(실측 또는 가상)로부터 centerline을 생성하고 DIRECT 모드 기본 경로로 사용한다.

* centerline 생성

  * `pair_valid=true`이면 `c_i = 0.5*(pL_i + pR_i)`다.
  * one-side+virtual이면 `c_i = p_vis + sgn*(w_hat/2)*n`이다.
  * centerline은 `resample_ds_center`로 리샘플하고 약한 smoothing을 적용할 수 있다.
  * yaw는 인접 점 벡터로 추정하고 unwrap을 적용한다.
* 출력 디버그(필수)

  * `/planning/debug/corridor_left`, `/planning/debug/corridor_right`다.
  * `/planning/debug/centerline`이다.
  * `/planning/debug/pair_valid`, `/planning/debug/virtual_used`다.
* `/planning/debug/corridor_polygon`은 생성하지 않는다.

---

### [OV-04-01-05] Costmap Build for Validation(DIRECT 공통)

**목표**: DIRECT 모드에서 centerline이 안전하게 통과 가능한지 빠르게 검증하기 위한 costmap을 구성한다.
**주의**: 여기서는 drivable mask를 만들지 않고 “경계/장애물 + inflation”만 구성한다.

* 차량 점 모델 및 inflation 정책

  * 차량을 점으로 모델링한다.
  * 충돌 판정을 차체와 동치로 만들기 위해 inflation 반경을 차량 폭 기반으로 둔다.
  * 기본 정책은 `r = vehicle_width/2 + safety_margin`다. 
  * 경계와 장애물에 서로 다른 margin을 줄 수 있다.

    * 예: `margin_boundary`, `margin_obstacle`를 둘 수 있다.
* 레이어 구조(DIRECT 검증용)

  1. Base/Unknown 레이어: 기본 unknown(-1)이다.
  2. Boundary Barrier 레이어: left/right 경계 polyline을 occupied(100)로 rasterize한다.
  3. Boundary Inflation 레이어: barrier를 `r_boundary`로 팽창시킨다.
  4. Obstacle 레이어: cone/obstacle 포인트를 occupied(100)로 rasterize한다.
  5. Obstacle Inflation 레이어: obstacle을 `r_obstacle`로 팽창시킨다.
* DIRECT 충돌 판정

  * centerline을 `ds_check` 간격으로 샘플링한다.
  * 각 샘플을 grid cell로 변환하고 다음 중 하나면 충돌이다.

    * cell이 occupied다.
    * (내부 cost 사용 시) cell cost가 `cost_th` 초과다.
* 핵심 운영 규칙

  * cone는 경계 구성에 사용되더라도 **충돌 금지 물체**이므로 obstacle 레이어에도 반드시 반영한다.
  * 동적 장애물도 obstacles로 들어오면 동일하게 obstacle 레이어에 반영한다.
* 파라미터 키(예)

  * `vehicle.width`다.
  * `safety.margin`, `safety.margin_boundary(선택)`, `safety.margin_obstacle(선택)`다. 
  * `costmap_valid.boundary.inflation_radius` 또는 `(vehicle.width/2 + margin_boundary)` 정책이다.
  * `costmap_valid.obstacle.inflation_radius` 또는 `(vehicle.width/2 + margin_obstacle)` 정책이다.
  * `costmap_valid.centerline.ds_check`, `costmap_valid.centerline.cost_th`다. 

---

### [OV-04-01-06] Drivable Mask Rasterize for A*(scanline fill)

**목표**: ASTAR 모드에서만 트랙 밖으로 새지 않도록 drivable mask를 scanline fill로 생성한다.
**주의**: polygon은 생성하지 않는다.

* 입력: `corridor_left[]`, `corridor_right[]`다.
* scanline fill 개념

  * grid의 각 x column(또는 진행좌표 s bin)마다 `y_left`와 `y_right`를 보간으로 얻는다.
  * `y_right ~ y_left` 사이 셀을 free(0)로 채운다.
  * 바깥 셀은 occupied(100) 또는 unknown(-1)로 둔다.
* 실패 게이트

  * 어떤 column에서든 `y_left <= y_right`이면(교차/뒤집힘) invalid다.
  * invalid면 ASTAR는 실행하지 않고 STOP/INFEASIBLE로 처리한다.
* ASTAR costmap 결합

  1. Drivable Mask 레이어를 base로 둔다.
  2. Obstacle 레이어 + inflation을 얹는다.
  3. (선택) Preference 레이어를 centerline 근처 비용 낮춤으로 줄 수 있다.
* obstacle inflation 반경 정책은 DIRECT 검증과 동일한 차량 점 모델 기반 정책을 공유한다. 

---

### [OV-04-01-07] Path Generation Mode Selector(최종)

**목표**: 매 tick마다 DIRECT(중앙선 기반) 또는 ASTAR(그리드 기반) 중 하나를 선택한다.

* 기본 정책은 DIRECT 기본이다.
* ASTAR 실행 트리거(아래 중 하나라도 만족)

  1. `pair_valid == false`다.
  2. `virtual_used == true`다(보수 정책 기본)다.
  3. `corridor_valid == false`다(scanline fill 실패 포함)다.
  4. centerline이 전방 `L_check` 내에서 validation costmap과 충돌한다.
  5. centerline이 프레임 간 과도하게 점프한다(`centerline_jump > centerline_jump_th`)다.
* DIRECT 선택 조건은 위 ASTAR 트리거가 모두 false인 경우다.
* 출력 정책

  * DIRECT: `path_raw = centerline`, `path_refined = postprocess(centerline)`다.
  * ASTAR: `path_raw = A* 결과`, `path_refined = postprocess(A* 결과)`다.
  * 디버그로 `path_mode`를 항상 발행한다.

---

### [OV-04-01-08] 출력 품질 관리

* 입력 토픽 스탬프가 오래되면 planner_status=STALE로 올리고 target_speed를 제한한다.
* corridor 생성 실패(좌/우 없음 + virtual 실패) 시 planner_status=INFEASIBLE로 올리고 target_speed를 0으로 수렴시킨다.
* DIRECT 모드에서 충돌이 반복되는데 ASTAR도 실패하면 planner_status=STOP으로 올리고 target_speed=0이다.

---

### [OV-04-02] Free-space 선택(ego connected component)

* 목표는 mask/costmap에서 여러 free 덩어리가 생길 때 ego가 실제 도달 가능한 영역만 사용하는 것이다.
* ego 셀을 seed로 BFS/DFS로 connected component를 추출한다.
* component 밖 free 셀은 제거한다.
* unknown은 기본적으로 탐색에서 제외한다.

---

### [OV-04-03] Goal Selection(v2: centerline 우선 + gated fallback)

* DIRECT 모드에서는 goal은 필수 입력이 아니며 centerline lookahead 점을 참조 목표(ref)로만 쓴다.
* ASTAR 모드에서는 goal이 필요하며 다음 정책을 사용한다.

#### [OV-04-03-01] Method1: centerline/path_prev 기반(기본)

* `path_prev`가 유효하면 경로 위 lookahead 거리 L만큼 앞 점 `g_ref`를 선택한다.
* `path_prev`가 없거나 불안정하면 centerline 위 lookahead L 점을 선택한다.
* 유효성 검사

  * ego component 내부다.
  * clearance 충분하다.
  * 셀 cost 과도하지 않다.
* 유효하면 goal=`g_ref`다.

#### [OV-04-03-02] Method2: free-space 링 샘플링(폴백)

* 실행 조건(게이트)

  * Method1 goal 무효다.
  * A* time budget 내 실패/미수렴이다.
  * corridor 품질이 낮다.
* 반경 R(=L) 링 위 후보 샘플링(예: 36~72개)다.
* 후보 점수는 clearance/progress/heading_align/proximity_to_ref를 사용한다.
* 최고점 후보를 goal로 선택한다.

---

### [OV-04-04] Planner(A*)

* A*는 Mode Selector가 ASTAR로 결정한 tick에서만 실행한다.
* DIRECT 모드에서는 A*를 실행하지 않는다.
* start는 ego cell이다.
* goal은 선택된 goal cell이다.
* cost는 기본 이동 비용 + cell_cost 반영이다.
* 게이트는 기존 path가 충분히 유효하면 재계획 생략 가능하고 goal 변화/막힘/코너에서만 재계획하는 것이다.

---

### [OV-04-05] Postprocess(Prune/Shortcut/Smooth + yaw)

* Prune: 중복/근접 점 제거다.
* Shortcut: 충돌 검사 통과 범위에서 불필요 꺾임 제거다.
* Smooth: 충돌 검사 기반 제한적 smoothing이다.
* yaw: 인접 점 벡터 기반 + unwrap이다.

---

### [OV-04-06] Feasibility/Safety(곡률·Rmin, STOP/INFEASIBLE, target_speed)

* 최소 회전반경 및 곡률 체크는 `R_min = L / tan(delta_max)`다.
* Feasibility

  * 경로 곡률 κ 계산 후 `|κ| ≤ 1/Rmin` 검사다.
  * 위반 시 INFEASIBLE이다.
  * (선택) 코너 구간 원호/Dubins 완화 + 충돌 체크를 시도할 수 있다.
  * 실패 시 STOP/재시도다.
* Safety(동적 장애물 포함)

  * 본 시스템은 차량을 점으로 모델링하며, 충돌 판정은 inflation된 costmap 기준으로 수행한다. 
  * DIRECT 모드에서 centerline이 전방 L_check 내 장애물/경계(inflation 포함)와 충돌하면 해당 tick은 ASTAR로 강제 전환한다. 
  * ASTAR에서도 경로 생성 실패 시 target_speed=0으로 정지한다. 
  * 정지거리 `d_stop` 여유가 없으면 STOP, target_speed=0 강제다.
  * Safety 결과는 Control/Driver에 최우선으로 반영한다.
* target_speed

  * 곡률 기반 제한 + 장애물 기반 제한을 최소로 적용한다.
  * 입력 스테일/플랜 오래됨이면 target_speed를 단계적으로 낮춰 0으로 수렴시킨다.

---

## [OV-05] 로컬 costmap(그리드) 사양

### [OV-05-01] 좌표계/프레임

* planning costmap은 ego 주변 로컬 프레임(`base_link` 정렬)을 기준으로 운용한다.
* RViz 시각화는 `map/odom` 변환과 함께 제공한다.

### [OV-05-02] 크기/해상도(권장 시작값)

* Frame: `base_link` 로컬이다.
* X 범위: `[-1m, +10m]`다.
* Y 범위: `[-4m, +4m]`다.
* 해상도: `0.10m`다.

### [OV-05-03] 셀 값 정의

* 출력: `nav_msgs/OccupancyGrid`다.
* free=0, occupied=100, unknown=-1이다.
* 내부 연산은 별도 cost 배열(uint8/uint16)로 유지 가능하다.

---

## [OV-06] 주기(Hz) 및 게이트/타임버짓 정책

### [OV-06-01] 권장 주기

* Perception/LiDAR 입력: 약 10Hz다.
* Planning tick: 10Hz다.
* Control tick: 50Hz 권장이다.
* Driver: control tick에 맞춰 publish하고 모드 명령은 상태 전환 시 호출한다.

### [OV-06-02] 타임버짓(예시)

* Planning 10Hz(100ms)에서 30ms 이내 목표다.

  * corridor build + pair/virtual + centerline: ≤ 8ms다.
  * validation costmap(boundary/obstacle rasterize + inflation): ≤ 5ms다.
  * (ASTAR 시) scanline mask rasterize + A*: ≤ 15ms다.
  * postprocess + safety: ≤ 2ms다. 

### [OV-06-03] 게이트 정책

* 입력 스테일 게이트

  * odom이 일정 시간 이상 갱신되지 않으면 감속→정지로 수렴한다.
  * lane/cone/obstacle 중 필수 입력이 오래되면 planner_status=STALE로 올리고 target_speed를 제한한다.
* 계산 게이트

  * A*는 time limit/iteration limit 초과 시 즉시 중단하고 “직전 경로 + 감속”으로 폴백한다.
* 출력 품질 게이트

  * pair invalid + virtual 실패면 곧바로 ASTAR/STOP로 수렴한다.
  * centerline/goal 점프는 히스테리시스로 완화한다.

---

## [OV-07] Driver 인터페이스 자동 탐색 및 연결 정책

### [OV-07-01] 추상 Driver 계약

* Control 표준 명령은 `ackermann_msgs/AckermannDriveStamped` 기반으로 고정한다.
* Driver는 표준 명령을 실차(CAN 등) 또는 시뮬 인터페이스로 변환한다.
* 차량 상태(속도/조향각/기어/에러)를 표준 토픽으로 제공한다.
* heartbeat/handshake로 연결 상태를 상시 보고한다.

### [OV-07-02] 자동 탐색(Discovery) 정책

* `driver_manager_node`가 ROS graph를 확인해 실차/시뮬 인터페이스 토픽 존재 여부를 탐색한다.
* 우선순위는 실차(핸드셰이크 OK) → 시뮬 → rosbag replay다.

### [OV-07-03] 연결 유지/장애 처리

* heartbeat 타임아웃 시 명령 송신 차단 + 정지 명령(가능 시)로 전환한다.
* driver 다운 시 전역 상태를 OFFLINE 또는 EMERGENCY_STOP로 올린다.

---

## [OV-08] 디버깅/단독 디버그 런치 운영 정책(track_bringup)

### [OV-08-01] track_bringup의 역할

* 운영 런치(실차/시뮬) 제공이다.
* 디버그 런치(플래닝 단독, 퍼셉션 단독, 제어 단독) 제공이다.
* rosbag record/replay 런치 제공이다.
* RViz 설정/디버그 오버레이 제공이다.

### [OV-08-02] 단독 디버그 런치(권장)

* `debug_planning_only.launch.py`는 bag/mock 입력으로 corridor/centerline/costmap/path를 주행 없이 검증한다.
* `debug_control_only.launch.py`는 준비된 path/target_speed로 제어를 검증한다.
* `replay_fullstack.launch.py`는 bag 기반 fullstack 재현이다.

### [OV-08-03] v2에서 필수 디버그 산출물

* `/planning/debug/corridor_left/right`다.
* `/planning/debug/centerline`다.
* `/planning/debug/pair_valid`, `/planning/debug/virtual_used`다.
* `/planning/debug/path_mode`다.
* `/planning/costmap_valid`(선택: RViz 확인용)이다.
* `/planning/costmap_astar`(선택)이다.

---

## [OV-09] 전역 상태 정의 및 manual/auto 연동 정책

### [OV-09-01] 전역 상태(enum) 정의(예시)

* OFFLINE, MANUAL, AUTO_STANDBY, AUTO_ACTIVE, AUTO_HOLD, INFEASIBLE, EMERGENCY_STOP다.

### [OV-09-02] 연동 정책

* MANUAL에서는 Control→Driver 명령을 차단한다.
* AUTO_ACTIVE 진입 조건은 driver OK, localization OK, E-STOP false다.
* AUTO_ACTIVE 중이라도 driver heartbeat loss 등 발생 시 EMERGENCY_STOP 또는 MANUAL로 전환한다.
* INFEASIBLE이 일정 시간 지속되면 AUTO_HOLD(정지)로 수렴한다.

---

## [OV-10] 최상위 I/O 계약(토픽/타입 요약)

### [OV-10-01] 코어 토픽 계약(요약)

| 구분              | 토픽                            | 타입                                            | 방향     | 비고                       |
| --------------- | ----------------------------- | --------------------------------------------- | ------ | ------------------------ |
| Localization    | `/localization/odom`          | `nav_msgs/Odometry`                           | In     | 필수 입력                    |
| Perception      | `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray`                | In     | 좌/우 분리 권장                |
| Perception      | `/perception/cones`           | `track_msgs/ConeArray`                        | In     | 경계 후보 + 장애물로도 사용         |
| Perception      | `/perception/obstacles`       | `track_msgs/ObstacleArray`                    | In     | 동적 장애물 포함                |
| Planning        | `/planning/costmap_valid`     | `nav_msgs/OccupancyGrid`                      | Out    | DIRECT 검증용(선택)           |
| Planning        | `/planning/costmap_astar`     | `nav_msgs/OccupancyGrid`                      | Out    | ASTAR용(선택)               |
| Planning        | `/planning/path_raw`          | `nav_msgs/Path`                               | Out    | centerline 또는 A* 결과      |
| Planning        | `/planning/path`              | `nav_msgs/Path`                               | Out    | postprocess 완료           |
| Planning        | `/planning/target_speed`      | `std_msgs/Float32`                            | Out    | m/s                      |
| Planning        | `/planning/status`            | `track_msgs/PlannerStatus`                    | Out    | OK/STOP/INFEASIBLE/STALE |
| Planning(Debug) | `/planning/debug/path_mode`   | `std_msgs/String` 또는 `track_msgs/PlannerMode` | Out    | DIRECT/ASTAR             |
| Control         | `/control/ackermann_cmd`      | `ackermann_msgs/AckermannDriveStamped`        | Out    | 표준 제어 명령                 |
| Driver          | `/vehicle/ackermann_cmd`      | (동일 또는 변환 후)                                  | Out    | 드라이버 입력                  |
| System          | `/system/state`               | `track_msgs/SystemState`                      | In/Out | 전역 상태                    |
| System          | `/system/engage`              | `std_msgs/Bool` 또는 서비스                        | In     | auto engage              |
| System          | `/system/estop`               | `std_msgs/Bool`                               | In     | 최우선                      |

### [OV-10-02] QoS 기본 정책(권장)

* 센서/인지 입력: BestEffort + KeepLast(1~5)다.
* 제어 명령: Reliable + KeepLast(1)다.
* 상태/진단: Reliable + KeepLast(10)다.

---

## [OV-11] Acceptance 기준(상위 레벨) v2

### [OV-11-01] 기능 수용 기준

* 차선-only에서 lane 기반 corridor 생성 후 주행이 가능해야 한다.
* 라바콘-only에서 cone 기반 corridor 생성 후 주행이 가능해야 한다.
* 혼합에서 lane+cone 공존 시 cone 우선 규칙으로 인터셉트 전이가 동작해야 한다.
* 코너에서 한쪽만 보일 때 virtual boundary로 centerline이 유지되어야 한다.
* 동적 장애물이 경로를 충분히 차단하면 STOP→target_speed=0→정지가 일관되게 동작해야 한다.
* 전방 부분 차단 장애물이 등장해 centerline이 충돌하면 path_mode가 ASTAR로 전환되어 회피 경로가 생성되어야 한다.

### [OV-11-02] 안전/보수 동작 수용 기준

* corridor 생성 실패 시 INFEASIBLE 또는 STOP으로 수렴해야 한다.
* pair invalid 상황에서 잘못된 경로를 강행하지 않아야 한다.
* 입력 스테일 시 감속/정지로 수렴해야 한다.
* (v2) 차량 점 모델 전제에서 inflation 반경을 `vehicle_width/2 + margin`로 설정했을 때 중앙선 충돌 판정이 차체 충돌과 동치로 동작해야 한다. 

### [OV-11-03] 성능/운영 수용 기준

* Planning 10Hz 유지 및 단계별 시간 로그 제공이 가능해야 한다.
* `track_bringup` 단독 디버그 런치에서 corridor/centerline/costmap/path를 재현 가능해야 한다.

---

# [CX-00] Codex CLI용 구현 명세(코드 생성 지시서)

## [CX-01] 구현 목표

* `track_planning` 내부에 “Corridor 기반 DIRECT/ASTAR 듀얼 모드 플래너”를 구현한다.
* 구현은 C++17, ROS2 Humble, rclcpp_components 기반 ComposableNode다.

---

## [CX-02] 파일/디렉터리 스켈레톤(권장)

* polygon 관련 파일을 제거하고 scanline mask/validation costmap 파일을 추가한다. 

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
    trajectory/
      astar.hpp
      postprocess.hpp
    manager/
      mode_selector.hpp
      safety_manager.hpp
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
    trajectory/
      astar.cpp
      postprocess.cpp
    manager/
      mode_selector.cpp
      safety_manager.cpp
    nodes/
      local_planner_node.cpp
  config/
    planning.yaml
  launch/
    planning_only.launch.py
```

---

## [CX-03] 핵심 클래스 계약(필수)

### [CX-03-01] CorridorBuilder

* 입력

  * `lane_left_pts, lane_right_pts`다.
  * `cone_pts`(또는 `cone_left/right_pts`)다.
  * `ego_pose`, `centerline_prev(optional)`다.
* 출력

  * `CorridorPolylines { left[], right[], left_ok, right_ok, DebugInfo dbg }`다.
* 구현 요구

  * seed는 side별 min x이며 `x>=x_seed_min` 필터를 적용한다.
  * CH-11 cone 우선 후보 풀을 적용한다.
  * CH-12 reference(c_end/t_end) 기반 s/d 필터를 적용한다.
  * CH-10-02 score 및 tie-break를 적용한다.
  * 종료 조건을 적용한다.

### [CX-03-02] PairValidator

* 입력: left/right polyline이다.
* 출력: `PairResult { bool valid; double width_median; double width_std; double angle_mean; }`다.
* 파라미터: `resample_ds, theta_mean_th, theta_max_th, w_min, w_max, w_std_th`다.

### [CX-03-03] VirtualBoundaryGenerator

* 입력: visible polyline + side + `w_hat`다.
* 출력: generated polyline + success flag다.
* 파라미터: `default_track_width, min_corridor_width, ema_alpha`다.

### [CX-03-04] CenterlineBuilder

* 입력: left/right polyline + `pair_valid` + virtual info(optional)다.
* 출력: `CenterlineResult { center[], bool valid }`다.
* 요구

  * pair_valid면 중앙값으로 생성한다.
  * virtual이면 offset으로 생성한다.
  * resample + smoothing + yaw 추정(선택)을 지원한다.

### [CX-03-05] CostmapValidationBuilder

* 입력: `corridor_left/right`, `cones`, `obstacles`, `vehicle_width`, `safety_margin`다.
* 출력: `nav_msgs::msg::OccupancyGrid`(validation) + 내부 cost(선택)다.
* 요구

  * boundary barrier rasterize다.
  * obstacle rasterize다.
  * inflation 반경에 `vehicle_width/2 + margin`을 반영한다. 
  * centerline 충돌 판정에 사용할 수 있어야 한다.

### [CX-03-06] DrivableMaskScanline

* 입력: `corridor_left/right`다.
* 출력: `nav_msgs::msg::OccupancyGrid`(mask) + valid flag다.
* 요구

  * polygon 없이 scanline fill로 mask 생성한다.
  * `y_left <= y_right` 발생 시 invalid 반환한다. 

### [CX-03-07] AStarPlanner

* 입력: mask + obstacle/inflation 포함 costmap, start cell, goal cell이다.
* 출력: grid 경로(point list)다.
* 요구

  * time limit/iteration limit가 있어야 한다.
  * 실패 시 실패 코드를 반환해야 한다.

### [CX-03-08] PostProcessor

* 입력: path raw다.
* 출력: path refined다.
* 요구: prune/shortcut/smooth/yaw unwrap이다.

### [CX-03-09] ModeSelector

* 입력: pair_valid, virtual_used, corridor_valid, centerline, validation costmap, centerline_jump다.
* 출력: `PlannerMode {DIRECT|ASTAR}`다.

### [CX-03-10] SafetyManager

* 입력: path_refined, costmap_valid(or astar), ego_speed, obstacle info다.
* 출력: `PlannerStatus`, `target_speed`다.
* 요구: 충돌/정지거리 여유/스테일 상태를 반영한다.

---

## [CX-04] Node 계약(LocalPlannerNode)

* 노드: `LocalPlannerNode`(ComposableNode)다.
* subscribe

  * `/localization/odom`
  * `/perception/lane_boundaries`
  * `/perception/cones`
  * `/perception/obstacles`
  * `/system/state`
* publish

  * `/planning/costmap_valid`(선택)
  * `/planning/costmap_astar`(선택)
  * `/planning/path_raw`, `/planning/path`
  * `/planning/target_speed`
  * `/planning/status`
  * `/planning/debug/*`(corridor/centerline/pair/virtual/path_mode)
* tick: 10Hz timer로 실행한다.
* debug publish에서 corridor_polygon은 제거한다. 

---

## [CX-05] planning.yaml 필수 파라미터 키(최소 템플릿)

```yaml
grid:
  resolution: 0.10
  roi:
    x_min: -1.0
    x_max: 10.0
    y_min: -4.0
    y_max: 4.0

vehicle:
  width: 1.20   # 예시다.

safety:
  margin: 0.20
  margin_boundary: 0.20   # 선택이다.
  margin_obstacle: 0.20   # 선택이다.

corridor:
  seed:
    x_seed_min: 0.0
  cold_start:
    min_centerline_points: 5
    theta_heading_gate_deg: 60.0
  ref_tangent:
    N_reg: 8
  side_filter:
    y_side_min_abs: 0.5   # 선택이다.
  filter:
    s_min: 0.2
    s_max: 1.6
    d_max: 1.5
    r_search: 2.0         # 선택이다.
  score:
    w_s: 2.0
    w_d: 3.0
    w_a: 1.0
    w_p: 1.0
    theta_max: 1.57
    step_pred: 1.0
  topk:
    enable: true
    K_top: 20
  limits:
    max_points_side: 200

centerline:
  resample_ds_center: 0.20
  smoothing_enable: true

mode_selector:
  enable_astar: true
  L_check: 6.0
  centerline_jump_th: 1.0

costmap_valid:
  centerline:
    ds_check: 0.10
    cost_th: 100
  boundary:
    inflation_radius: -1  # -1이면 vehicle.width/2 + safety.margin_boundary 정책을 쓴다.
  obstacle:
    inflation_radius: -1  # -1이면 vehicle.width/2 + safety.margin_obstacle 정책을 쓴다.

costmap_astar:
  mask:
    outside_mode: occupied   # occupied 또는 unknown이다.
  obstacle:
    inflation_radius: -1     # vehicle.width/2 + margin_obstacle 정책이다.
  preference:
    enable: false

goal:
  lookahead_L0: 2.0
  lookahead_kv: 0.3
  ring_samples: 60

timeouts:
  odom_ms: 100
  perception_ms: 250
  plan_ms: 30
```

* inflation은 “차량 점 모델 반영”으로 명시한다. 

---

## [CX-06] 최소 테스트 시나리오(rosbag 기반)

* lane-only bag: pair_valid=true, virtual_used=false, path_mode=DIRECT 유지다.
* cone-only bag: pair_valid=true, path_mode=DIRECT 유지다.
* mixed intercept bag: lane 대신 cone로 경계 전이되고 path 유지다.
* corner one-side bag: pair_valid=false → virtual_used=true → DIRECT 또는 필요 시 ASTAR다.
* partial block obstacle bag: centerline 충돌 → ASTAR 전환 → 회피 성공 또는 실패 시 정지다.
* full block obstacle bag: DIRECT 충돌 → ASTAR 실패 → STOP, target_speed=0이다.

---

원하면 위 최종 명세서를 기준으로 `track_msgs` 메시지 초안(.msg)과 `planning_only.launch.py`, 그리고 Codex CLI가 바로 생성할 수 있는 **파일별 스텁 코드 템플릿(TODO 주석 포함)**까지 한 번에 확장해 줄 수 있다.
