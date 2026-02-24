[OV-00] 문서 범위와 전제

- 본 문서는 Linux + ROS2 Humble 기반 자율주행 대회 출전을 위한 **확장성 있는 베이스 시스템 기획서(overview)**다.
- 트랙은 **차선-only / 라바콘-only / 차선+라바콘 혼합(차선이 라바콘으로 차단되어 cone 트랙으로 전이) / 동적 장애물(진입 시 정지 필요)**로 구성된다는 전제를 둔다.
- 사용자가 개별 답변을 추적 가능하도록 본 문서 모든 주요 섹션에 **고정 ID**를 부여한다.
- 문체는 목적지향적·팩트기반이며 문장 종결은 “-다”로 통일한다.

---

## [OV-01] 목표와 운영 정책

### [OV-01-01] 시스템 목표

- 트랙 유형이 바뀌어도(차선↔라바콘↔혼합) **동일한 베이스 로직**에서 입력(Perception)만 달라져도 주행이 지속되도록 설계한다.
- 동적 장애물 트랙에서 장애물이 진입하면 **안전 정지(브레이크 포함)**를 수행하고, 장애물이 해제되면 재출발(대회 룰 허용 시)을 수행할 수 있는 구조로 만든다.
- 실차/시뮬/rosbag 재생 환경에서 동일한 노드 조합을 사용하고, 파라미터(YAML)로 동작을 바꿀 수 있게 한다.
- 장애/오류(인지 누락, 시간 초과, 경로 미생성)가 발생하면 **Fail-safe: 감속→정지**로 수렴하도록 한다.

### [OV-01-02] 운영 정책(설계 원칙)

- **모듈 분리 원칙**: 인지/플래닝/제어/드라이버를 토픽 계약(I/O contract)으로만 결합하고, 내부 구현은 독립적으로 교체 가능하게 한다.
- **시간 예산(Time budget) 우선**: “최적”보다 “기한 내 계산 + 안전한 보수 동작”을 우선한다.
- **스테일 데이터 정책**: 입력이 오래되면(지연/끊김) 곧바로 “추정 주행”을 하지 않고 감속 또는 정지로 전환한다.
- **디버그 우선 정책**: 모든 주요 단계는 RViz/로그/디버그 토픽을 제공하며, 단독 런치로 재현 가능해야 한다.
- **파라미터화**: 트랙/속도/차량제원(휠베이스, Rmin, 가감속 한계) 및 게이트 임계값을 YAML로 분리한다.

---

## [OV-02] 전체 시스템 파이프라인(Perception → Planning → Control → Driver)

### [OV-02-01] 상위 데이터 흐름

아래는 “대회용 최소 의존성 + 확장성”을 목표로 한 표준 흐름이다.

```
Sensors/Localization
  ↓
Perception (lane, cones, obstacles, drivable hints)
  ↓
Planning (local costmap → free-space → goal → A* → postprocess → safety)
  ↓
Control (path tracking + speed control + command shaping)
  ↓
Driver (interface adapter + health/handshake + vehicle I/O)
  ↓
Actuators / Vehicle / Simulator
```

### [OV-02-02] 트랙 유형별 처리 관점(동일 파이프라인 유지)

- 차선-only: Lane 경계/중심 기반으로 drivable 영역을 만들고, goal은 차선 진행방향 기반으로 선택한다.
- 라바콘-only: 콘을 장애물/경계로 해석해 drivable corridor를 만들고, goal은 free-space 전방 최대 진행점 기반으로 선택한다.
- 혼합: 차선이 존재해도 콘이 차선을 차단할 수 있으므로, **goal 선택 단계에서 “차선 기반(Method1)”이 막히면 “free-space 기반(Method2)”로 폴백**한다.
- 동적 장애물: 장애물 감지 시 Planning의 Safety 단계가 **STOP/EMERGENCY_STOP**를 출력하고, Control/Driver가 즉시 정지 커맨드를 우선 적용한다.

### [OV-02-03] 전역 상태(Global state)와 파이프라인 결합

- Planning 결과가 정상이어도 전역 상태가 MANUAL이면 제어 명령을 내보내지 않는다.
- 전역 상태가 AUTO_ACTIVE일 때만 Control→Driver 명령이 전달되며, 그 외 상태에서는 “0 속도/중립” 또는 “driver pass-through”로 정책을 분기한다.

---

## [OV-03] C++ 컴포넌트(ComposableNode) 아키텍처 개요 + 패키지/노드 책임 분리

### [OV-03-01] ComposableNode 아키텍처 개요

- 각 기능 노드는 `rclcpp_components::NodeFactory`로 등록된 **ComposableNode**로 구현한다.
- 운영 모드에 따라 두 가지 실행 방식을 제공한다.
    - **Composition 모드(권장 운영)**: 하나의 component container 프로세스에 여러 노드를 탑재해 intra-process 통신(메모리 복사 최소화)과 배포 단순화를 확보한다.
    - **Decomposition 모드(권장 디버그)**: 각 노드를 별도 프로세스로 실행해 gdb/로그 분리를 쉽게 한다.
- Executor 정책은 다음을 기본으로 한다.
    - 제어/드라이버는 지연에 민감하므로 별도 callback group 또는 별도 container로 격리한다.
    - Planning은 time budget이 중요하므로 단일 스레드(결정론) 또는 제한된 멀티스레드(콜백 분리)를 선택한다.

### [OV-03-02] 패키지 구조

- `track_bringup`
    - launch, config, vehicle param, rviz config, rosbag/replay 스크립트 포함
- `track_msgs`
    - Cone/Lane/Obstacle/SystemState/PlannerStatus 등 커스텀 메시지 정의(최소 의존성)
- track_planning(folder, not a package)
    
    ├──`costmap builder` : costmap builder, freespace, goal selection 로직
    
    ├──`trajectory build` : A*, postprocess
    
    └──`local_planner_manager` :safety/feasibility
    
- `track_control`
    - path tracker(횡제어), speed controller(종제어), command shaping(제한/필터), control mux
- `track_driver`
    - 차량 인터페이스 어댑터, handshake/health, 자동 탐색(driver discovery)

### [OV-03-03] 노드 책임 분리(핵심 규칙)

- Perception은 “무엇이 보이느냐”만 말하고, Planning은 “어디로 갈 것이냐”만 말한다.
- Planning은 costmap/grid 기반으로 통일해 입력 표현이 바뀌어도(차선/콘/혼합) 동일한 A*와 후처리를 재사용한다.
- Control은 path + target_speed만 보고 동작하며, Perception의 디테일을 직접 참조하지 않는다.
- Driver는 어떤 하드웨어/시뮬이 오더라도 “동일한 제어 명령 계약”을 만족시키도록 변환한다.

---

## [OV-04] Planning 로컬 파이프라인

### [OV-04-00] Planning 입력/출력 정의(로컬 플래닝 관점)

- 입력(대표)
    - `ego_pose` / `ego_twist` (odometry 기반)
    - `lane_boundaries`
    - `cones`
    - `system_state`(AUTO_ACTIVE 여부, E-STOP 여부)
- 출력(대표)
    - `local_costmap`
    - `path_raw`, `path_refined`
    - `planner_status`(OK/STOP/INFEASIBLE/STALE 등)
    - `target_speed`(m/s) 및 제한 사유 코드

---

### [OV-04-01] 4.1 Costmap Builder

**목표**: 차선/라바콘/장애물 정보를 하나의 “로컬 그리드 비용장(cost field)”로 통합한다.

- 입력 융합 정책
    - 라바콘/장애물: occupancy(장애물 셀) + inflation(완충 비용)로 반영한다.
    - 차선: “차선 내부는 free 혹은 낮은 비용”, “차선 외부는 unknown 또는 높은 비용”으로 반영한다. 차선이 없으면 해당 레이어는 비활성화한다.
- 레이어 구조(권장)
    1. **Base/Unknown 레이어**: 기본은 unknown(-1)로 두고, 관측/추정 가능한 영역만 free로 연다.
    2. **Drivable 레이어(차선 기반)**: 차선 신뢰도가 높을 때만 활성화한다.
    3. **Obstacle 레이어(콘/장애물)**: 점/클러스터를 grid에 rasterize한다.
    4. **Inflation 레이어**: 차량 폭, 위치 오차, 제어 오차를 흡수하도록 장애물 주변 비용을 증가시킨다.
    5. **Preference 레이어(선호 비용)**: 차선 중앙/콘 코리더 중앙을 선호하도록 중심선 방향 비용 편향을 줄 수 있다.
- 출력 품질 관리
    - 입력 토픽 스탬프가 오래되면(스테일) costmap 갱신을 중단하거나, costmap을 “위험 상태”로 마킹해 후단에서 감속/정지로 유도한다.

---

### [OV-04-02] 4.2 Free-space 선택(ego connected component)

**목표**: costmap의 free 셀이 여러 덩어리로 분리될 때, 실제로 차량이 도달 가능한 영역만 사용한다.

- 핵심 아이디어
    - ego가 위치한 셀을 seed로 하여 BFS/DFS로 **connected component**를 추출한다.
    - 추출된 연결 free-space 외의 free 셀(센서 노이즈로 생긴 “떠 있는 섬”)은 planning에서 제외한다.
- 기대 효과
    - 혼합 트랙에서 차선 레이어와 콘 레이어가 충돌해 costmap이 끊어져도, “현재 도달 가능한 영역” 기준으로 goal/경로 탐색이 안정화된다.
- 구현 포인트
    - 2D 그리드에서 4-neighbor 또는 8-neighbor 연결성을 선택한다.
    - 비용이 낮아도 “unknown”은 기본적으로 연결 탐색에서 제외하여 보수적으로 운용한다.

---

### [OV-04-03] 4.3 Goal Selection(Method1 + gated Method2)

**목표**: 트랙이 차선 기반이든 콘 기반이든, 일관된 방식으로 “전방 목표점”을 선택한다.

### Method1: 규칙 기반(차선/참조 진행 기반)

- `path_prev`가 있을 때, 경로 위에서 lookahead 거리 L만큼 앞 점 `g_ref` 선택
- `g_ref` 유효성 검사:
    - ego 컴포넌트 내부
    - clearance 충분
    - 셀 cost 과도하지 않음
- 유효하면 goal = `g_ref` (경로 안정성↑)
- 입력 조건(예)
    - 차선 인지가 존재하며 신뢰도/연속성이 임계값 이상이다.
    - 최근 N프레임에서 차선 방향이 급변하지 않는다.
- 목표 생성(예)
    - ego 기준 lookahead 거리 `L`만큼 전방으로 진행하는 지점(차선 중심선 또는 차선 방향 벡터 기반)을 goal로 둔다.
    - lookahead는 속도에 따라 가변(`L = L0 + k*v`)로 두어 고속에서 과도한 조향을 줄인다.

### Method2: free-space 기반(차선 부재/차단 대응)

- 사용 목적
    - 라바콘-only 트랙, 차선이 가려진 구간, 또는 혼합 트랙에서 **차선이 라바콘에 의해 차단(intercept)**되어 Method1 goal이 장애물/금지 영역에 걸리는 경우에 사용한다.
- `g_ref`가 무효/불안정일 때만 실행(게이트)
- 반경 R(=L) 링 위에 후보들을 샘플링(예: 36~72개)
- 후보 점수:
    - clearance(장애물에서 멀수록)
    - progress(전진성)
    - heading_align(통로 진행 방향 근사)
    - **proximity_to_ref** (가능하면 `g_ref` 근처 선호)
- 최고점 후보를 goal로 선택
- gated 정책(“필요할 때만 Method2”)
    - Method1이 다음 중 하나라도 만족하면 Method2로 폴백한다.
        - Method1 goal 셀이 occupied/inflated 영역이다.
        - Method1 goal이 ego connected component 밖이다.
        - Method1 기반 A*가 time budget 내 실패/미수렴이다.
    - Method2는 다음 조건을 만족해야 채택한다.
        - goal 후보가 충분히 큰 free-space 내부(주변 free 비율 임계값)다.
        - 직전 goal 대비 yaw 변화량이 임계값 이하(목표 흔들림 억제)다.
- 혼합 트랙 대응 포인트
    - “차선이 존재한다”는 사실만으로 Method1을 고집하지 않고, **goal 유효성(costmap 상 안전성)**을 최우선으로 gate한다.

---

### [OV-04-04] 4.4 Planner(A*)

**목표**: 그리드 costmap에서 장애물을 회피하며 goal까지 도달하는 경로를 생성한다.

- 탐색 그래프
    - 2D grid, 8-neighborhood(대각 이동 허용) 권장이다.
    - 이동 비용은 (기본 이동거리) + (셀 비용 가중치) + (조향/곡률 페널티)로 구성한다.
- 휴리스틱
    - Euclidean 또는 Octile distance를 사용한다.
- 비용 설계(대회 맞춤)
    - 장애물 inflation 영역은 “통과는 가능하나 비용이 매우 큰” 형태로 두면, 좁은 구간에서 완전 막힘을 피하면서도 최대한 안전하게 중앙을 타게 만들 수 있다.
    - 차선/콘 중앙 선호를 preference 레이어로 주면, 경로가 지그재그로 흔들리는 현상을 줄일 수 있다.
- 시간 예산 준수
    - iteration limit 및 time limit를 둔다.
    - time budget 초과 시: “직전 유효 경로 유지 + 목표 속도 감속”으로 폴백한다.
- start: ego cell
- goal: 위에서 선택한 goal cell
- cost: 기본 이동 비용 + cell_cost(soft) 반영
- 게이트:
    - 기존 path가 충분히 유효하면 재계획 생략 가능
    - goal 큰 변화/막힘/코너 진입 등에서만 재계획

---

### [OV-04-05] 4.5 Postprocess(Prune/Shortcut/Smooth + yaw 추정)

**목표**: A*의 계단형 경로를 제어 가능한 연속 경로로 변환한다.

- Prune
    - 시작부에서 ego와 과도하게 가까운 점, 중복점 제거로 경로 길이를 정리한다.
- Shortcut
    - 충돌 검사(collision check) 통과하는 범위 내에서 불필요한 꺾임을 제거한다.
- Smooth
    - 단순 moving average는 코너에서 바깥으로 새어 나갈 수 있으므로, costmap 충돌 검사 기반의 제한적 smoothing을 권장한다.
    - 출력 포인트 간격(resample)을 일정하게 맞춰 제어기의 입력 품질을 안정화한다.
- yaw 추정
    - 각 포인트의 yaw는 인접 점 벡터로 계산하고, 저속/정지 구간에서는 직전 yaw를 유지한다.
    - yaw의 unwrap(연속성 유지)을 적용해 ±π 경계 점프를 방지한다.

---

### [OV-04-06] 4.6 Feasibility/Safety(곡률·Rmin, STOP/INFEASIBLE, target_speed)

**목표**: “갈 수 있는 경로”인지, “가도 안전한지”를 최종 판정하고 속도를 결정한다.

- 최소 회전반경 및 곡률 체크
    - `R_min = L / tan(delta_max)`
- Feasibility(기구학/차량제원)
    - 경로 곡률 κ를 계산해 `|κ| ≤ 1/Rmin`을 만족하는지 검사한다.
    - 위반 시 상태를 **INFEASIBLE**로 두고, 대체 경로 탐색(재플랜)을 트리거한다.
- 위반 시 처리 정책(운영 규칙)
    1. **빠른 수정 시도(선택):** 코너 구간만 원호/Dubins 스타일로 완화 + 충돌 체크
    2. 그래도 실패하면: 경로를 “없음”으로 보고 STOP/재시도
- Safety(동적 장애물/정지)
    - 전방 장애물과의 거리, 상대 속도(가능 시)를 이용해 최소 정지거리 `d_stop` 대비 여유를 계산한다.
    - 여유가 없으면 **STOP**을 출력하고 `target_speed = 0`을 강제한다.
    - 동적 장애물 트랙 요구사항은 “인지→플래닝→제어” 어디에서도 누락되지 않게, Safety 결과가 Control/Driver에 **최우선**으로 반영되게 한다.
- target_speed 결정(예시 정책)
    - 곡률 기반 제한: `v_max_curve = sqrt(a_lat_max / |κ|)` 형태로 횡가속 제한을 준다.
    - 장애물 기반 제한: 전방 여유거리 기반으로 `v_max_obs`를 둔다.
    - 최종: `target_speed = min(v_cmd_mission, v_max_curve, v_max_obs)`로 결정한다.
    - 입력 스테일/플랜 오래됨이면 `target_speed`를 단계적으로 낮춰 결국 0으로 수렴시킨다.

---

## [OV-05] 로컬 costmap(그리드) 사양

### [OV-05-01] 좌표계/프레임

- planning costmap은 ego 주변 로컬 프레임(권장: `odom` 또는 `base_link` 정렬 프레임)에서 운용한다.
- RViz 시각화는 `map/odom` 변환과 함께 제공하되, 로컬 플래닝 자체는 **로컬 일관성**을 우선한다.

### [OV-05-02] 크기/해상도(권장 시작값, 파라미터화 전제)

- Frame: `base_link` 로컬
- X 범위: **[-1m, +10m]** (총 11m)
- Y 범위: **[-4m, +4m]** (총 8m)
- 해상도: **0.10m**

### [OV-05-03] 셀 값 정의(표준 호환)

- 출력 메시지는 RViz/디버그 호환을 위해 `nav_msgs/OccupancyGrid`를 기본으로 두되, 내부 연산은 별도 cost 배열(uint8/uint16)을 사용해도 된다.
- OccupancyGrid 관례를 따른다.
    - free: 0
    - occupied: 100
    - unknown: -1
- inflation/preference 등 “연속 비용”은 내부 costmap에서 0~255로 운용하고 시각화 시 스케일링해 내보낸다.

---

## [OV-06] 주기(Hz) 및 게이트/타임버짓 정책

### [OV-06-01] 권장 주기(초기안)

- Perception/LiDAR 입력: 약 **10Hz**
- Planning tick: **10Hz**
    - **Goal 업데이트는 별도 타이머 없이 planning tick 내부에서 수행**
- Control tick(Tracking): **50Hz** 권장
- Driver:
    - ControlCommand는 control tick에 맞춰 publish
    - ModeCommand는 시작 시 1회(또는 상태 전환 시) 호출

### [OV-06-02] 타임버짓(예시)

- Planning 10 Hz 기준(주기 100 ms)에서 목표 CPU 예산을 30 ms 내로 두는 보수 정책 예시다.
    - Costmap build: ≤ 8 ms
    - Connected component: ≤ 2 ms
    - Goal selection: ≤ 2 ms
    - A*: ≤ 15 ms (초과 시 중단/폴백)
    - Postprocess: ≤ 3 ms
    - Feasibility/Safety: ≤ 2 ms

### [OV-06-03] 게이트(계산/데이터) 정책

- 입력 스테일 게이트
    - odom이 일정 시간(예: 100 ms) 이상 갱신되지 않으면 AUTO_ACTIVE를 유지하지 않고 감속→정지로 수렴시킨다.
    - lane/cone/obstacle 중 “필수 입력”이 오래되면(예: 200~300 ms) planner_status를 STALE로 올리고 target_speed를 제한한다.
- 계산 게이트
    - costmap은 ego 이동량(예: 0.2 m) 또는 perception 업데이트가 있을 때만 갱신하도록 최적화할 수 있다.
    - A*는 time limit/iteration limit를 넘기면 즉시 중단하고, “직전 경로 + 감속”으로 폴백한다.
- 안정성 게이트(출력 품질)
    - goal이 프레임마다 급변하면 yaw-rate가 폭증하므로, goal 변화량 제한(저역필터/히스테리시스)을 적용한다.
    - 경로가 짧거나(예: 최소 포인트 수 미달) 안전 검사를 통과하지 못하면 STOP/INFEASIBLE로 전환한다.

---

## [OV-07] Driver 인터페이스 자동 탐색 및 연결 정책

### [OV-07-01] 추상 Driver 계약(상위)

- Control이 내보내는 “표준 명령”은 하나로 고정한다. 예: `ackermann_msgs/AckermannDriveStamped` 기반이다.
- Driver는 다음을 책임진다.
    - 표준 명령을 실차(CAN 등) 또는 시뮬 인터페이스로 변환한다.
    - 차량 상태(속도/조향각/기어/에러)를 표준 토픽으로 제공한다.
    - heartbeat/handshake로 연결 상태를 상시 보고한다.

### [OV-07-02] 자동 탐색(Discovery) 정책

- `driver_manager_node`가 ROS graph를 확인해 다음을 탐색한다.
    - “실차 드라이버가 publish하는 상태 토픽” 존재 여부
    - “실차 드라이버가 subscribe하는 명령 토픽” 존재 여부
    - 시뮬레이터 인터페이스 토픽 존재 여부
- 우선순위 정책(예)
    1. 실차 드라이버(핸드셰이크 OK)
    2. 시뮬 드라이버
    3. rosbag replay 모드(명령 송신 비활성 또는 검증 모드)

### [OV-07-03] 연결 유지/장애 처리

- heartbeat 타임아웃 시 즉시 “명령 송신 차단 + 정지 명령(가능 시)”로 전환한다.
- driver가 다운되면 시스템 전역 상태를 OFFLINE 또는 EMERGENCY_STOP로 올려 Planning/Control이 보수 동작을 하게 한다.

---

## [OV-08] 디버깅/단독 디버그 런치 운영 정책(track_bringup 집중)

### [OV-08-01] track_bringup의 역할

- 대회 운영에서 가장 중요한 것은 “같은 조합으로 언제든 재현”이다. `track_bringup`은 다음을 제공한다.
    - 운영 런치(실차/시뮬)
    - 디버그 런치(플래닝 단독, 퍼셉션 단독, 제어 단독)
    - rosbag record/replay 런치
    - RViz 설정/디버그 오버레이(marker, costmap view, path view)

### [OV-08-02] 단독 디버그 런치(권장 세트)

- `debug_planning_only.launch.py`
    - 입력: rosbag 또는 mocked perception(정적 테스트 데이터)
    - 출력: costmap/path/status/target_speed + RViz marker
    - 목적: A*, postprocess, safety를 “주행 없이” 검증한다.
- `debug_control_only.launch.py`
    - 입력: 미리 준비된 path + target_speed(또는 bag)
    - 출력: ackermann_cmd 및 제어 상태 그래프
- `replay_fullstack.launch.py`
    - bag를 재생하면서 전체 스택을 돌려, 실제 주행과 동일하게 재현한다.

### [OV-08-03] 운영 정책(로그/성능)

- 모든 노드는 최소한 다음을 로그로 남긴다.
    - 입력 스테일 여부, 계획 성공/실패, time budget 초과, STOP/INFEASIBLE 사유 코드
- 성능 측정을 위해 planning 각 단계의 실행 시간을 토픽 또는 `/diagnostics` 형태로 발행하는 정책을 권장한다.

---

## [OV-09] 전역 상태 정의 및 manual/auto 연동 정책

### [OV-09-01] 전역 상태(enum) 정의(예시)

- OFFLINE: 드라이버 미연결/필수 입력 불가
- MANUAL: 수동 운전, 자율 명령 미적용
- AUTO_STANDBY: 자율 준비 완료, 아직 engage 전
- AUTO_ACTIVE: 자율 제어 적용 중
- AUTO_HOLD: 일시 보류(입력 스테일/경로 불안정)로 감속 또는 정지 유지
- INFEASIBLE: 경로 생성 불가(재시도 중)
- EMERGENCY_STOP: 즉시 정지 상태(최우선)

### [OV-09-02] 연동 정책(상호작용 규칙)

- MANUAL에서는 Planning은 실행해도 되지만(상태 추적용), Control→Driver 명령은 **차단**한다.
- AUTO_ACTIVE는 다음 조건을 만족할 때만 진입한다.
    - driver 연결 OK(heartbeat OK)
    - localization OK(최근 갱신)
    - E-STOP false
- AUTO_ACTIVE 중이라도 다음 발생 시 즉시 EMERGENCY_STOP 또는 MANUAL로 전환한다.
    - 운전자 override 신호(가능 시)
    - driver heartbeat loss
    - Safety STOP 조건(동적 장애물 포함) 지속
- INFEASIBLE은 일정 횟수/시간 내 회복 못하면 AUTO_HOLD(정지)로 수렴시킨다.

---

## [OV-10] 최상위 I/O 계약(토픽/타입 요약)

### [OV-10-01] 코어 토픽 계약(요약)

아래는 “최상위 인터페이스” 중심 요약이며, 실제 이름은 네임스페이스(`/track/...`)로 정리하는 것을 권장한다.

| 구분 | 토픽 | 타입 | 방향 | 비고 |
| --- | --- | --- | --- | --- |
| Localization | `/localization/odom` | `nav_msgs/Odometry` | In | 필수 입력 |
| Perception | `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray`(예) | In | 선택 입력(없을 수 있음) |
| Perception | `/perception/cones` | `track_msgs/ConeArray`(예) | In | 선택 입력(없을 수 있음) |
| Perception | `/perception/obstacles` | `track_msgs/ObstacleArray`(예) | In | 동적 장애물 포함 |
| Planning | `/planning/costmap` | `nav_msgs/OccupancyGrid` | Out | 디버그/검증 핵심 |
| Planning | `/planning/path_raw` | `nav_msgs/Path` | Out | A* 원본 |
| Planning | `/planning/path` | `nav_msgs/Path` | Out | 후처리 완료 |
| Planning | `/planning/target_speed` | `std_msgs/Float32` | Out | m/s |
| Planning | `/planning/status` | `track_msgs/PlannerStatus`(예) | Out | OK/STOP/INFEASIBLE/STALE |
| Control | `/control/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Out | 표준 제어 명령 |
| Driver | `/vehicle/ackermann_cmd` | (동일 또는 변환 후) | Out | 실제 드라이버 입력 |
| Driver | `/vehicle/status` | `track_msgs/VehicleStatus`(예) | In/Out | 속도/조향/에러 |
| System | `/system/state` | `track_msgs/SystemState`(예) | In/Out | 전역 상태 |
| System | `/system/engage` | `std_msgs/Bool` 또는 서비스 | In | auto engage |
| System | `/system/estop` | `std_msgs/Bool` | In | E-STOP(최우선) |

### [OV-10-02] QoS 기본 정책(권장)

- 센서/인지 입력: BestEffort + KeepLast(1~5)로 지연 최소화를 우선한다.
- 제어 명령: Reliable + KeepLast(1)로 최신 명령 보장을 우선한다.
- 상태/진단: Reliable + KeepLast(10)로 이력 추적을 우선한다.

---

## [OV-11] Acceptance 기준(상위 레벨)

### [OV-11-01] 기능 수용 기준

- 차선-only 구간에서 차선 기반(Method1) goal로 정상 주행이 가능해야 한다.
- 라바콘-only 구간에서 cone 기반(Method2) goal로 정상 주행이 가능해야 한다.
- 혼합 구간에서 차선이 콘에 의해 차단될 때 Method1이 실패하면 **자동으로 Method2로 전환**되어 주행이 지속되어야 한다.
- 동적 장애물 트랙에서 장애물 진입 시 **STOP 상태 출력 → target_speed=0 → 차량 정지**가 일관되게 동작해야 한다.

### [OV-11-02] 안전/보수 동작 수용 기준

- 입력 스테일(odom/perception) 상황에서 속도가 유지/증가하지 않고 감속 또는 정지로 수렴해야 한다.
- 경로가 Rmin 제약을 위반하면 INFEASIBLE로 판단하고, 무리한 조향 명령을 내지 않아야 한다.
- planner time budget 초과 시에도 “마지막 유효 경로 + 감속”으로 안전하게 동작해야 한다.

### [OV-11-03] 성능/운영 수용 기준

- Planning 루프가 설정된 Hz에서 지속 동작하며, 단계별 실행 시간이 로그/진단으로 관측 가능해야 한다.
- `track_bringup`의 단독 디버그 런치로 rosbag 기반 재현이 가능해야 한다.
- 실차/시뮬 모드 전환이 런치 파라미터/driver discovery 정책으로 가능해야 한다.

---

원하면 다음 단계로 [OV-04] 로컬 플래닝 파이프라인을 기준으로 **구체 파라미터 세트(YAML 템플릿), 메시지 정의(track_msgs 초안), track_bringup 런치 트리(파일 구조)**까지 한 번에 내려서 바로 구현 가능한 형태로 정리할 수 있다.