[OV-00] 문서 범위와 전제 v2

* 본 문서는 Linux + ROS2 Humble 기반 자율주행 대회 출전을 위한 **확장성 있는 베이스 시스템 기획서(overview)**다. 
* 트랙은 **차선-only / 라바콘-only / 차선+라바콘 혼합(차선이 라바콘으로 차단되어 cone 트랙으로 전이) / 동적 장애물(진입 시 정지 필요)**로 구성된다는 전제를 둔다. 
* 사용자가 개별 답변을 추적 가능하도록 본 문서 모든 주요 섹션에 **고정 ID**를 부여한다. 
* 문체는 목적지향적·팩트기반이며 문장 종결은 “-다”로 통일한다. 
* v2 변경점의 핵심은 Planning의 drivable 생성 방식이다.

  * 기존: costmap 상 low-cost를 따라 goal/경로를 안정화하는 방식 중심이다.
  * 변경(v2): **차선/라바콘 포인트로 좌/우 경계 polyline을 greedy chaining으로 구성하고**, 이로부터 corridor polygon 및 centerline을 만든 뒤, 그 polygon을 그리드에 rasterize하여 drivable을 만든다.

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
      - boundary barrier rasterize + boundary inflation
      - obstacle(cones/obstacles) rasterize + inflation
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

* 차선-only: 차선 포인트 기반으로 좌/우 경계 polyline을 만들고 corridor polygon을 생성한다.
* 라바콘-only: 콘 포인트 기반으로 좌/우 경계 polyline을 만들고 corridor polygon을 생성한다.
* 혼합: 좌/우 체인 구성 시 “전방 1m 근처에 차선과 라바콘이 함께 존재하면 라바콘을 우선 채택”하여, 차선 트랙이 콘으로 intercept 되는 경우 자연스럽게 콘 경계로 전이한다.
* 동적 장애물: 장애물 감지 시 Safety가 STOP/EMERGENCY_STOP를 출력하고, Control/Driver가 즉시 정지 커맨드를 우선 적용한다.

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

  * `costmap_builder/` : corridor build + pair/virtual + polygon rasterize + free-space + goal
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
  * `corridor_polygon` (디버그 polygon marker)
  * `centerline` (디버그/goal 참조)
  * `path_raw`, `path_refined`
  * `planner_status`(OK/STOP/INFEASIBLE/STALE 등)
  * `target_speed`(m/s) 및 제한 사유 코드

---

### [OV-04-01] 4.1 Corridor 기반 Costmap Builder(v2 핵심)

**목표**: 차선/라바콘 포인트를 “경계”로 해석하여 corridor polygon을 만들고, polygon inside를 drivable로 rasterize하여 costmap을 구성한다.
---

## [OV-04-01-01] 좌/우 corridor 배열 생성(Left/Right chaining) v2.1

### [OV-04-01-01-A] Reference(중앙차선 기준) 정의(채택: CH-12)

* chaining의 “전방/가까움” 기준은 **중앙차선(centerline)의 끝점과 접선**을 기준으로 정의한다.
* reference point는 `c_end`(중앙차선 끝점)이다.
* reference tangent는 `t_end`(중앙차선 진행 방향 단위벡터)이다.
* `t_end`는 마지막 2점이 아니라 **마지막 N점(권장 5~10점) 방향 회귀**로 계산한다.

  * 구현 예: 마지막 N점에 대해 평균 방향 벡터를 만들고 정규화한다.
  * 또는 PCA/선형회귀로 1차 주성분 방향을 구해 정규화한다.
* centerline이 아직 없거나 신뢰도가 낮으면 다음 순서로 fallback한다.

  1. 직전 프레임 `centerline_prev`의 `c_end/t_end`다.
  2. 직전 프레임 경계선(좌/우) 중 유효한 쪽의 마지막 접선이다.
  3. ego heading(+x)이다.
#### [OV-04-01-01-B] Cold Start(초기 프레임) 기준 c_end/t_end 결정 규칙(추가)

* 정의: cold start는 아래 중 하나를 만족하는 상태다.

  1. centerline이 아직 생성되지 않았다.
  2. centerline은 존재하나 길이가 min_centerline_points 미만이다.
  3. pair_valid=false이고 virtual_used=false로 corridor 품질이 매우 낮다.

* cold start에서 reference는 다음 순서로 결정한다.

  1. 이전 프레임 centerline_prev가 존재하면
       * c_end = centerline_prev.back()이다.
       * t_end = regress_tangent(centerline_prev, N_reg)다.

  2. 이전 centerline이 없고, 이번 프레임에 좌/우 경계 중 하나라도 seed가 잡혔으면
       * c_end = ego_position이다.
       * t_end = normalize(seed - ego_position)이다.
       * 단, dot(t_end, ego_heading) < cos(theta_heading_gate)이면(너무 옆/뒤면) 이 경로는 폐기하고 3)으로 간다.

  3. 위 두 조건이 모두 불가하면
       * c_end = ego_position이다.
       * t_end = ego_heading(+x)이다.

* 파라미터 키(추가)
    * corridor.cold_start.min_centerline_points
    * corridor.cold_start.theta_heading_gate_deg

* 목적
    * 초기 1~2프레임에서 중앙차선이 없더라도 chaining의 s/d 필터 및 점수화가 안정적으로 동작하게 만드는 데 있다.

### [OV-04-01-01-C] Candidate Pool 구성(채택: CH-11)

* 매 스텝 k에서 직전 경계 포인트 `p_k`가 주어졌을 때, 탐색 범위 안에서 후보를 먼저 모은다.
* 후보 집합을 source 별로 나눈다.

  * `C_cone`: cone 후보들이다.
  * `C_lane`: lane 후보들이다.
* **cone 우선 규칙**은 점수화 전에 적용한다.

  * 필터를 통과한 cone 후보가 1개라도 있으면 **후보 풀은 `C_cone`만 사용**한다.
  * cone 후보가 없으면 후보 풀은 `C_lane`만 사용한다.
* 목적은 “cone이 있으면 lane을 아예 보지 않는다”를 명확히 구현하는 데 있다.

### [OV-04-01-01-D] 후보 필터링(채택: CH-10, 1단계: Filter)

* 후보는 단순 거리 원(1m)만 쓰지 않고, 중앙차선 진행좌표 기반으로 필터링한다.
* 각 후보 점 `p`에 대해 다음을 계산한다.

  * 전방 진행량: `s = dot(p - c_end, t_end)`다.
  * 접선에 대한 횡거리: `d = abs(cross2(t_end, p - c_end))`다.
* 필터 조건(파라미터화)

  * 전방성: `s >= s_min`이다.
  * 전방 탐색 상한: `s <= s_max`다.
  * 횡방향 제한: `d <= d_max`다.
  * (선택) 원형 거리 제한: `dist(p, p_k) <= r_search`다.
* 필터 목적은 “뒤/옆 라인 점프”를 제거하고, 중앙차선 진행 방향에 정렬된 후보만 남기는 데 있다.

### [OV-04-01-01-E] 후보 점수화 및 선택(채택: CH-10-02, 2단계: Scoring)

* 필터를 통과한 후보 풀에서 최종 1개를 선택한다.
* 기본은 **점수 최대(score max)** 선택이다.
* 점수는 “멀수록(전방 진행량 큼) + 중앙차선 접선에 가까울수록(횡거리 작음)”을 강하게 반영한다.
* 점수 구성(예시, 파라미터화)

  * `score(p) = + w_s * norm_s(s) - w_d * norm_d(d) - w_a * norm_a(Δθ) - w_p * norm_p(pred_err)`다.
  * `Δθ = angle_diff(t_k, p - p_k)`다.
  * `pred_err = dist(p, p_pred)`다.
  * `p_pred = p_k + step_pred * t_k`다.
  * `t_k`는 직전 두 경계 포인트로 만든 접선이며, 초기에는 `t_end` 또는 ego heading을 사용한다.
* 정규화 예시

  * `norm_s(s) = clamp(s / s_max, 0, 1)`이다.
  * `norm_d(d) = clamp(d / d_max, 0, 1)`이다.
  * `norm_a(Δθ) = clamp(Δθ / theta_max, 0, 1)`이다.
  * `norm_p(pred_err) = clamp(pred_err / r_search, 0, 1)`이다.
* 선택 규칙

  * `score`가 가장 큰 후보를 채택해 `p_{k+1}`로 둔다.

### [OV-04-01-01-F] “접선에 가까운 후보 우선” Top-K 최적화(선택)

* 후보 수가 많을 때 연산을 줄이고 싶으면 다음을 적용한다.

  * 필터 통과 후보를 `d` 오름차순으로 정렬한다.
  * 상위 `K`개만 남긴다(`K_top` 파라미터).
  * 그 `K`개에 대해서만 [OV-04-01-01-E] 점수 계산을 수행한다.
* 이 방법은 “접선에 가까운 점 우선”을 보장하면서도, score 기반으로 안정적으로 고르게 한다.

### [OV-04-01-01-G] 종료 조건(유지)

* 다음 조건 중 하나를 만족하면 체인을 종료한다.

  * 다음 후보가 없을 때다.
  * 포인트 수가 `max_points_side`를 초과할 때다.
  * x가 ROI 끝(`x_max`)에 도달했을 때다.

### [OV-04-01-01-H] 파라미터 키(추가/정리)

* Seed

  * `corridor.seed.x_seed_min`
* Filter

  * `corridor.filter.s_min`, `corridor.filter.s_max`
  * `corridor.filter.d_max`
  * `corridor.filter.r_search`(선택)
* Scoring

  * `corridor.score.w_s`, `corridor.score.w_d`, `corridor.score.w_a`, `corridor.score.w_p`
  * `corridor.score.theta_max`
  * `corridor.score.step_pred`
* Top-K

  * `corridor.topk.enable`, `corridor.topk.K_top`
* Regression tangent

  * `corridor.ref_tangent.N_reg`

---

#### [OV-04-01-02] 좌/우 쌍(pair) 평행성/폭 검증

**목표**: 좌/우 경계가 “서로 다른 라인”을 따라가거나 교차하는 경우를 감지해 드라이빙 실패를 예방한다.

* 입력: `corridor_left[]`, `corridor_right[]`
* 전처리

  * 두 polyline을 동일 간격(`resample_ds`)으로 resample한다.
  * 각 샘플에서 tangent를 구한다.
* 평행성 검사

  * 각 샘플 i에 대해 `Δθ_i = angle(tL_i, tR_i)`를 구한다.
  * 조건 예: `mean(Δθ_i) < theta_mean_th` AND `max(Δθ_i) < theta_max_th`
* 폭 검사

  * 각 샘플 i에서 폭 `w_i`를 계산한다.
  * 조건 예: `w_min < median(w_i) < w_max` AND `std(w_i) < w_std_th`
* 교차/뒤집힘 검사

  * 폭이 음수로 뒤집히거나 polyline 교차가 감지되면 pair invalid로 둔다.
* 결과

  * pair_valid=true이면 centerline/corridor polygon을 “실측 좌/우”로 생성한다.
  * pair_valid=false이면 [OV-04-01-03] 가상 차선 생성 로직으로 넘어간다.

#### [OV-04-01-03] 한쪽만 보일 때 가상 차선(virtual boundary) 생성

**목표**: 코너 등에서 한쪽 경계만 인식되는 경우에도 corridor와 centerline을 생성해 주행을 유지한다.

* 입력 케이스

  * left만 존재, right 없음
  * right만 존재, left 없음
  * 또는 pair_valid=false지만 한쪽이 상대적으로 품질이 더 좋다고 판단되는 경우
* 폭 추정값 `w_hat` 정책

  1. 직전 프레임에서 pair_valid=true였을 때의 `median(w_i)`를 우선 사용한다.
  2. 직전 값이 없으면 설정값 `default_track_width`를 사용한다.
  3. `w_hat`는 프레임 간 급변하지 않도록 저역통과(EMA)로 갱신한다.
* 가상 경계 생성 수식

  * 보이는 경계가 `P_vis`일 때, 각 점에서 접선 `t` 및 법선 `n=rot90(t)`를 구한다.
  * 가상 점 `p_virtual = p_vis + sgn * w_hat * n` 로 생성한다.
  * `sgn`은 좌/우 side에 따라 결정한다(좌가 보이면 우로, 우가 보이면 좌로 이동한다).
* 가상 경계 품질 게이트

  * corridor 최소 폭이 `vehicle_width + margin`보다 작으면 실패 처리한다.
  * 가상 경계가 ROI 밖으로 크게 벗어나면 실패 처리한다.
  * (선택) 콘/장애물 occupancy와 충돌이 과도하면 실패 처리한다.
* 결과

  * 가상 경계 생성 성공 시 centerline을 생성한다.
  * 실패 시 cone 기반만으로 구성된 경계 또는 free-space 기반 goal로 폴백한다.

#### [OV-04-03.5] Path Generation Mode Selector

* 목적: 매 tick마다 Direct Path(중앙선 기반) 또는 A Path(그리드 기반)* 중 하나를 선택한다.
* 기본 정책: Direct Path가 기본이며, 아래 조건에서만 A*를 실행한다.

A* 실행 트리거(아래 중 하나라도 만족)
1. pair_valid == false다.
2. virtual_used == true다.
3. corridor_valid == false다(폴리곤 생성 실패 포함).
4. 중앙선 경로가 전방 L_check 내에서 obstacle/inflation과 충돌한다.
5. min_width(s) < min_corridor_width가 전방 L_check 내에 존재한다.
6. 중앙선이 프레임 간 과도하게 점프한다(centerline_jump > th_jump).
   
Direct Path 선택 조건
*  위 A* 트리거 조건이 모두 false이면 Direct Path를 사용한다.

출력 정책
* Direct Path 선택 시: path_raw = centerline, path_refined = postprocess(centerline)다.
* A* 선택 시: path_raw = A* 결과, path_refined = postprocess(A* 결과)다.
* 디버그로 path_mode를 항상 발행한다(DIRECT 또는 ASTAR).
  
파라미터 키(추가)
* mode_selector.L_check
* mode_selector.centerline_jump_th
* mode_selector.enable_astar(기본 true)
  
#### [OV-04-01-04] centerline 생성
* centerline(direct mode)
  
  * pair_valid=true면 `c_i = 0.5*(pL_i+pR_i)`로 만든다.
  * one-side+virtual이면 `c_i = p_vis + sgn*(w_hat/2)*n`로 만든다.
* 출력 디버그

  * `/planning/debug/centerline`
  * `/planning/debug/pair_valid`, `/planning/debug/virtual_used`
  * 
#### [OV-04-01-04.5] Corridor polygon 생성
* corridor polygon(direct mode fallback)
  * A* 실행이 트리거 됐을때 만든다.
  * `P_corridor = poly( left_polyline + reverse(right_polyline) )` 형태로 닫는다.
  * self-intersection이 감지되면 simplify 후 재검사한다.
* 출력 디버그
  * `/planning/debug/corridor_left`, `/planning/debug/corridor_right`
  * `/planning/debug/corridor_polygon`

#### [OV-04-01-05] Grid rasterize 및 costmap 레이어(v2)

* 기본 사상

  * corridor polygon 내부는 free(0)로 연다.
  * corridor polygon 외부는 occupied(100) 또는 unknown(-1)로 둔다(운영 정책에 따라 선택한다).
* 레이어 구조(v2 권장)

  1. **Base/Unknown 레이어**: 기본은 unknown(-1)로 둔다.
  2. **Drivable Mask 레이어**: corridor inside를 free(0), outside를 occupied(100)로 둔다.
  3. **Obstacle 레이어**: 콘 포인트 및 장애물을 occupied로 rasterize한다.
  4. **Inflation 레이어**: obstacle 주변을 팽창시켜 여유를 강제한다.
  5. **Preference 레이어(선택)**: centerline으로부터의 lateral distance 기반 비용을 부여해 중앙을 선호하게 한다.
* 핵심 운영 규칙
  * costmap rasterize는 A* 모드에서 필수, Direct Path 모드에서는 디버그/충돌체크용으로 선택으로 둔다.
  * Drivable mask rasterize 및 A*용 costmap 구성은 기본적으로 필요 시(A* 모드) 수행한다.
  * Direct Path 모드에서도 충돌체크를 위해 obstacle/inflation만 최소 costmap으로 유지할 수 있다(구현 선택).
  * cone는 경계 구성에 사용되더라도 **충돌 금지 물체**이므로 obstacle 레이어에도 반드시 반영한다.
  * corridor가 좁아 inflation으로 막히는 경우가 발생할 수 있으므로 inflation radius는 트랙 폭과 함께 튜닝 대상이다.
  
* 파라미터 키(추가, 선택)
  * costmap.enable_in_direct_mode(기본 false 또는 true는 구현 선택)

#### [OV-04-01-06] 출력 품질 관리

* 입력 토픽 스탬프가 오래되면(스테일) costmap 갱신을 중단하거나 planner_status=STALE로 올린다.
* corridor 생성 실패(좌/우 없음 + virtual 실패) 시 planner_status=INFEASIBLE로 올리고 target_speed를 0으로 수렴시킨다.

---

### [OV-04-02] 4.2 Free-space 선택(ego connected component)

**목표**: corridor rasterize 과정에서 생기는 끊김/구멍이 있더라도 ego가 실제 도달 가능한 free-space만 사용한다.

* ego 셀을 seed로 BFS/DFS로 connected component를 추출한다.
* connected component 밖의 free 셀은 제거한다.
* unknown은 기본적으로 탐색에서 제외한다(보수적).

---

### [OV-04-03] 4.3 Goal Selection(v2: centerline 우선 + gated fallback)
* Direct Path 모드에서는 goal은 경로 생성을 위한 필수 입력이 아니며, centerline 기반 lookahead 점을 “참조 목표(ref)”로만 사용한다.
* A* 모드에서는 기존과 동일하게 goal을 선택하고 A*의 목표 셀로 사용한다.

**목표**: corridor 기반 drivable에서 안정적으로 전방 목표점을 선택한다.

#### [OV-04-03-01] Method1: centerline/path_prev 기반(기본)

* `path_prev`가 유효하면 경로 위 lookahead 거리 L만큼 앞 점 `g_ref`를 선택한다.
* `path_prev`가 없거나 불안정하면 centerline 위에서 lookahead L만큼 앞 점을 선택한다.
* goal 유효성 검사

  * ego connected component 내부
  * clearance 충분
  * 셀 cost 과도하지 않음
* 유효하면 goal=`g_ref`로 둔다.

#### [OV-04-03-02] Method2: free-space 링 샘플링(폴백)

* 실행 조건(게이트)

  * Method1 goal이 무효(occupied/inflated, component 밖, 스테일)
  * A*가 time budget 내 실패/미수렴
  * corridor 품질이 낮음(pair invalid + virtual 실패 등)
* 반경 R(=L) 링 위 후보 샘플링(예: 36~72개)
* 후보 점수

  * clearance
  * progress
  * heading_align
  * proximity_to_ref(가능하면 `g_ref` 근처 선호)
* 최고점 후보를 goal로 선택한다.

---

### [OV-04-04] 4.4 Planner(A*)

* A*는 Mode Selector가 ASTAR로 결정한 tick에서만 실행한다.
* DIRECT 모드에서는 A*를 실행하지 않고 centerline을 경로로 사용한다.
* start: ego cell
* goal: 선택된 goal cell
* cost: 기본 이동 비용 + cell_cost 반영(Preference 레이어를 쓰면 여기서 반영된다)
* 게이트

  * 기존 path가 충분히 유효하면 재계획을 생략 가능하다.
  * goal 큰 변화/막힘/코너 진입 등에서만 재계획한다.

---

### [OV-04-05] 4.5 Postprocess(Prune/Shortcut/Smooth + yaw)

* Prune: 중복/근접 점 제거
* Shortcut: 충돌 검사 통과 범위에서 불필요 꺾임 제거
* Smooth: 충돌 검사 기반의 제한적 smoothing
* yaw 추정: 인접 점 벡터 기반 + unwrap 적용

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

  * DIRECT 모드에서 centerline 경로가 전방 L_check 내 장애물(inflation 포함)과 충돌하면, 해당 tick은 ASTAR 모드로 강제 전환한다.
  * ASTAR 모드에서도 경로 생성 실패 시 target_speed=0으로 정지한다.
  * 전방 장애물 대비 정지거리 `d_stop` 여유가 없으면 STOP, target_speed=0 강제
  * Safety 결과는 Control/Driver에 최우선으로 반영한다.
* target_speed

  * 곡률 기반 제한 + 장애물 기반 제한을 최소로 적용한다.
  * 입력 스테일/플랜 오래됨이면 target_speed를 단계적으로 낮춰 0으로 수렴시킨다.

---

## [OV-05] 로컬 costmap(그리드) 사양

### [OV-05-01] 좌표계/프레임

* planning costmap은 ego 주변 로컬 프레임(`base_link` 정렬)을 기준으로 운용한다.
* RViz 시각화는 `map/odom` 변환과 함께 제공한다.

### [OV-05-02] 크기/해상도(v2 고정)

* Frame: `base_link` 로컬
* X 범위: **[-1m, +10m]**
* Y 범위: **[-4m, +4m]**
* 해상도: **0.10m**

### [OV-05-03] 셀 값 정의

* 출력: `nav_msgs/OccupancyGrid`
* free=0, occupied=100, unknown=-1
* 내부 연산은 별도 cost 배열(uint8/uint16)로 유지 가능하다.

---

## [OV-06] 주기(Hz) 및 게이트/타임버짓 정책

### [OV-06-01] 권장 주기(초기안)

* Perception/LiDAR 입력: 약 **10Hz**
* Planning tick: **10Hz**
* Control tick: **50Hz**
* Driver: control tick에 맞춰 publish, 모드 명령은 상태 전환 시 호출

### [OV-06-02] 타임버짓(예시)

* Planning 10Hz(100ms)에서 30ms 이내 목표

  * corridor build + pair/virtual + polygon: ≤ 8ms
  * rasterize: ≤ 5ms
  * connected component: ≤ 2ms
  * goal selection: ≤ 2ms
  * A*: ≤ 15ms
  * postprocess + safety: ≤ 3ms

### [OV-06-03] 게이트 정책(v2 추가)

* 입력 스테일 게이트: 동일
* 계산 게이트: 동일
* 출력 품질 게이트(v2)

  * pair_valid=false이고 virtual도 실패하면 곧바로 Method2 또는 STOP으로 수렴한다.
  * corridor 최소 폭이 `min_corridor_width` 미만이면 goal/path를 무효로 처리한다.
  * centerline/goal 점프가 임계값을 넘으면 히스테리시스로 완화한다.

---

## [OV-07] Driver 인터페이스 자동 탐색 및 연결 정책

(기존과 동일)

---

## [OV-08] 디버깅/단독 디버그 런치 운영 정책(track_bringup 집중)

### [OV-08-01] v2에서 추가해야 하는 디버그 산출물

* corridor build 디버그 토픽을 필수로 제공한다.

  * left/right polyline
  * pair_valid, virtual_used
  * corridor polygon, centerline
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
  * `/planning/debug/corridor_polygon` : `visualization_msgs/Marker`
  * `/planning/debug/pair_valid` : `std_msgs/Bool`
  * `/planning/debug/virtual_used` : `std_msgs/Bool`
  * `/planning/debug/path_mode` : `std_msgs/String` 또는 `track_msgs/PlannerMode`(예)
    * 값: `DIRECT` / `ASTAR`다.

---

## [OV-11] Acceptance 기준(상위 레벨) v2 추가

### [OV-11-01] 기능 수용 기준

* 차선-only에서 lane 기반 corridor 생성 후 주행이 가능해야 한다.
* 라바콘-only에서 cone 기반 corridor 생성 후 주행이 가능해야 한다.
* 혼합에서 “전방 1m 근처 lane+cone 공존 시 cone 우선” 규칙으로 인터셉트 전이가 필터링되어야 한다.
* 코너에서 한쪽 차선만 보일 때 virtual boundary 생성으로 centerline/goal이 유지되어야 한다.
* 동적 장애물 진입 시 STOP→target_speed=0→정지가 일관되게 동작해야 한다.
* 정상 구간에서는 path_mode=DIRECT가 유지되어야 한다.
* 전방 부분 차단 장애물이 등장해 centerline이 충돌하면 path_mode가 ASTAR로 전환되어 회피 경로가 생성되어야 한다(회피 불가 시 정지).

### [OV-11-02] 안전/보수 동작 수용 기준

* corridor 생성 실패 시 INFEASIBLE 또는 STOP으로 수렴해야 한다.
* pair invalid(교차/폭 비정상) 상황에서 잘못된 경로를 강행하지 않아야 한다.
* 입력 스테일 시 감속/정지로 수렴해야 한다.

### [OV-11-03] 성능/운영 수용 기준

* Planning 10Hz 유지 및 단계별 시간 로그 제공이 가능해야 한다.
* `track_bringup` 단독 디버그 런치에서 corridor/centerline/costmap/path를 재현 가능해야 한다.

---

# [CX-00] Codex CLI용 구현 명세(코드 생성 지시서)

## [CX-01] 구현 목표

* `track_planning` 내부에 “Corridor 기반 drivable rasterize”를 구현한다.
* 구현은 C++17, ROS2 Humble, rclcpp_components 기반 ComposableNode로 구성한다.

## [CX-02] 파일/디렉터리 스켈레톤(권장)

```
track_planning/
  include/track_planning/
    corridor/
      corridor_builder.hpp
      pair_validator.hpp
      virtual_boundary.hpp
      corridor_polygon.hpp
    costmap/
      costmap_rasterizer.hpp
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
      corridor_polygon.cpp
    costmap/
      costmap_rasterizer.cpp
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

* 입력: `std::vector<Point2D> lane_pts, cone_pts`(좌/우 분리된 버전도 함께)
* 출력: `CorridorPolylines { std::vector<Point2D> left, right; bool left_ok, right_ok; DebugInfo dbg; }`
* 주요 파라미터

  * `x_seed_min, r_search, x_forward_min, x_forward_max, max_points_side`
  * `cone_priority_enable=true`
* 구현 요구

  * seed = min x
  * forward 1m 탐색
  * lane+cone 공존 시 cone 우선

### [CX-03-02] PairValidator

* 입력: left/right polyline
* 출력: `PairResult { bool valid; double width_median; double width_std; double angle_mean; }`
* 파라미터: `resample_ds, theta_mean_th, theta_max_th, w_min, w_max, w_std_th`

### [CX-03-03] VirtualBoundaryGenerator

* 입력: visible polyline + side + `w_hat`
* 출력: generated polyline + success flag
* 파라미터: `default_track_width, min_corridor_width, ema_alpha`

### [CX-03-04] CorridorPolygonBuilder

* 입력: left/right polyline
* 출력: polygon(점열), centerline(점열), validity
* 요구: self-intersection 검사 + simplify 옵션

### [CX-03-05] CostmapRasterizer

* 입력: corridor polygon, cones, obstacles
* 출력: `nav_msgs::msg::OccupancyGrid` + 내부 cost 배열(선택)
* 요구:

  * inside free / outside occupied(or unknown)
  * cones/obstacles occupied
  * inflation optional
  * preference optional(distance-to-centerline)

### [CX-03-06] ConnectedComponent

* 입력: OccupancyGrid, ego cell index
* 출력: mask 또는 filtered grid

### [CX-03-07] GoalSelector

* 입력: centerline/path_prev + connected component + costmap
* 출력: goal point + method used + score

## [CX-04] Node 계약(로컬 플래너 노드)

* 노드: `LocalPlannerNode`(ComposableNode)
* subscribe

  * `/localization/odom`
  * `/perception/lane_boundaries`
  * `/perception/cones`
  * `/perception/obstacles`
  * `/system/state`
* publish

  * `/planning/costmap`
  * `/planning/path_raw`, `/planning/path`
  * `/planning/target_speed`
  * `/planning/status`
  * `/planning/debug/*` (corridor/centerline/pair/virtual)
* tick: 10Hz timer로 실행한다.

## [CX-05] planning.yaml 필수 파라미터 키(예시)

* `roi.x_min, roi.x_max, roi.y_min, roi.y_max, grid.resolution`
* `corridor.seed.x_min`
* `corridor.chain.r_search, corridor.chain.x_forward_min, corridor.chain.x_forward_max, corridor.chain.max_points`
* `pair.resample_ds, pair.theta_mean_th, pair.theta_max_th, pair.w_min, pair.w_max, pair.w_std_th`
* `virtual.default_track_width, virtual.ema_alpha, virtual.min_corridor_width`
* `inflation.radius, inflation.cost`
* `goal.lookahead_L0, goal.lookahead_kv, goal.ring_samples`
* `timeouts.odom_ms, timeouts.perception_ms, timeouts.plan_ms`

## [CX-06] 단위 테스트/시나리오 테스트(최소)

* lane-only bag: pair_valid=true, virtual_used=false, path 생성
* cone-only bag: pair_valid=true, path 생성
* mixed intercept bag: cone 우선 채택 구간에서 lane 대신 cone로 경계가 전이되고 path가 유지
* corner one-side bag: pair_valid=false → virtual_used=true → centerline 유지 → path 생성
* dynamic obstacle bag: STOP 상태 및 target_speed=0 출력 확인

---

원하는 형태가 “문서만”이 아니라, Codex CLI가 바로 작업할 수 있게 **작업 단위(Task list) + 함수 시그니처 + TODO 주석 포함 템플릿 코드**까지 포함한 형태라면, 위 [CX-02~06]을 기반으로 “파일별 스텁 코드”까지 풀어서 작성할 수 있다.
