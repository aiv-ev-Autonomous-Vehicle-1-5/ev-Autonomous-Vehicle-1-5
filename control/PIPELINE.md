# Control Package Pipeline (pp_controller_cpp)

## 개요

T870 차량을 위한 **상대좌표 Pure Pursuit 경로 추종 제어 노드**.
Planning 모듈이 생성한 base_link 기준 상대좌표 경로(Marker POINTS)를 입력받아,
기하학적 경로 추종 알고리즘(Pure Pursuit)으로 **속도 적응형 lookahead**, **곡률 기반 속도 제어**, **rate-limited 가감속**을 적용하여
조향각과 속도를 계산하고 T870 실차 및 ERP42 Gazebo 시뮬레이터에 전달한다.

---

## 패키지 구조

```
control/
├── CMakeLists.txt                     # 빌드 설정
├── package.xml                        # 패키지 메타데이터 (pp_controller_cpp)
├── PIPELINE.md                        # 이 문서
├── topic.md                           # 토픽 인터페이스 요약
├── config/
│   └── pure_pursuit.yaml              # ROS 2 파라미터 설정
├── launch/
│   └── pure_pursuit.launch.py         # 런치 파일
└── src/
    └── pure_pursuit_relative_node.cpp # 전체 노드 구현 (단일 파일)
```

### 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 클라이언트 라이브러리 |
| `visualization_msgs` | 경로 입력 (Marker POINTS) + 디버그 시각화 (Marker SPHERE / LINE_STRIP) |
| `t870_msgs` | T870 실차 제어 명령 (`ControlCommand`: speed, steering) |
| `erp42_msgs` | ERP42 Gazebo 시뮬레이션 제어 명령 (`ControlCommand`: speed, steering, brake) |

---

## 전체 시스템 내 위치

```
[Perception] → [Planning] → [Control] → [Vehicle Interface]
                              ^^^^^^^^
                              이 패키지
```

```
velodyne_points (PointCloud2)        camera (LaneBoundaryArray)
  → Patchwork++ (지면 분리)                │
  → DBSCAN (클러스터링)                     │
  → MakeBBox (BBox 생성)                   │
  → /perception/bboxes (BBoxArray)         │
          │                                │
          └──────────────┬─────────────────┘
                         ▼
  → DirectionChainer (좌/우 경계 체인 생성)
  → CostmapGenerator (Gaussian 비용 필드 생성)
  → A* Planner (8방향 격자 최단 경로 탐색)
  → PostProcessor (smooth + resample + curvature clamp)
  → SafetyChecker (곡률 검증)
  → /planning/path (Marker POINTS)              ← 이 노드의 입력
  → /planning/status (String)                   ← FAIL 시 정지 조건
  → PurePursuitRelativeNode (조향 + 속도 계산)   ← 이 노드
  → /t870/control_command (ControlCommand)       ← 실차 출력
  → /erp42/control_command (ControlCommand)      ← Gazebo 시뮬레이션 출력 (lazy)
  → /pp_debug/lookahead_point (Marker SPHERE)    ← 디버그: lookahead 목표점 (lazy)
  → /pp_debug/pursuit_arc (Marker LINE_STRIP)    ← 디버그: PP 원호 궤적 (lazy)
```

---

## 토픽 인터페이스

### 입력 (Subscribe)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | BestEffort, depth=10 | base_link 기준 상대좌표 경로. 각 Point의 x는 전방(+), y는 좌측(+). |
| `/planning/status` | `std_msgs/msg/String` | BestEffort, depth=10 | planning 상태. FAIL 시 정지 (`not enough seeds` / `no valid path` / `too short valid path`). |

#### Marker(POINTS) 메시지 상세

```
visualization_msgs/msg/Marker
├── header
│   ├── stamp        # 경로 생성 시각
│   └── frame_id     # "base_link" (상대좌표)
├── type             # Marker::POINTS
└── points[]         # geometry_msgs/Point 배열
    ├── x            # 차량 전방 거리 [m] (앞이 +)
    ├── y            # 차량 횡방향 거리 [m] (좌가 +)
    └── z            # 미사용 (0)
```

### 출력 (Publish)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | BestEffort, depth=1 | T870 실차 제어 명령 (항상 발행) |
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | depth=10 | ERP42 Gazebo 시뮬레이션 제어 명령 (lazy — 구독자 있을 때만) |

### 디버그 출력 (Publish, lazy — 구독자가 있을 때만 발행)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/pp_debug/lookahead_point` | `visualization_msgs/msg/Marker` (SPHERE) | BestEffort, depth=1 | PP가 선택한 lookahead 목표점 (초록 구, base_link) |
| `/pp_debug/pursuit_arc` | `visualization_msgs/msg/Marker` (LINE_STRIP) | BestEffort, depth=1 | PP 예상 원호 궤적 (노란 선, base_link). kappa≈0이면 직선. |

#### T870 ControlCommand 메시지

```
t870_msgs/msg/ControlCommand
├── speed     # float64: 목표 속도 [m/s]
└── steering  # float64: 조향각 [rad] (좌회전 +, 우회전 -)
```

#### ERP42 ControlCommand 메시지

```
erp42_msgs/msg/ControlCommand
├── speed     # float64: 목표 속도 [m/s]
├── steering  # float64: 조향각 [rad] (좌회전 +, 우회전 -)
└── brake     # uint8:   브레이크 [0~1] (정지 시 1, 주행 시 0)
```

---

## 노드 내부 파이프라인

```
┌──────────────────────────────────────────────────────────────────┐
│                   PurePursuitRelativeNode                        │
│                                                                  │
│  /planning/path ──→ [on_path 콜백]                               │
│                        │                                         │
│                        ▼                                         │
│                   latest_points_ 저장                             │
│                   last_path_time_ 기록                            │
│                                                                  │
│  20Hz 타이머 ──→ [on_timer 제어루프]                               │
│                        │                                         │
│                   ①  path_fresh() 확인                            │
│                        │  (경로 비어있거나 timeout → 정지)          │
│                        ▼                                         │
│                   ①-b planning status 확인                        │
│                        │  (FAIL 시 정지: not enough seeds /        │
│                        │   no valid path / too short valid path)  │
│                        ▼                                         │
│                   ②  find_nearest_index()                        │
│                        │  - 경로 전체에서 원점(0,0)에 최근접점 탐색  │
│                        ▼                                         │
│                   ③  compute_preview_curvature()                 │
│                        │  - preview_distance(2.5m) 내 최대 곡률    │
│                        ▼                                         │
│                   ④  compute_speed_target()                      │
│                        │  - v = √(a_lat / κ_preview)              │
│                        │  - clamp(v, v_min, v_max)                │
│                        ▼                                         │
│                   ⑤  compute_dynamic_lookahead()                 │
│                        │  - Ld = Ld_min + gain × speed            │
│                        │  - clamp(Ld, Ld_min, Ld_max)             │
│                        ▼                                         │
│                   ⑥  compute_target_relative()                   │
│                        │  - 최근접점부터 누적 arc length로          │
│                        │    lookahead 목표점 선택                  │
│                        ▼                                         │
│                   ⑦  안전 조건 확인                                │
│                        │  - Ld < 1e-3 → 정지                      │
│                        │  - tx ≤ min_x_target → 정지              │
│                        ▼                                         │
│                   ⑧  Pure Pursuit 조향각 계산                     │
│                        │  kappa = 2*y / Ld²                       │
│                        │  delta = atan(L * kappa)                 │
│                        │  delta = clamp(delta, ±delta_max)        │
│                        ▼                                         │
│                   ⑨  최종 속도 결정                                │
│                        │  effective_κ = max(|κ_pp|, κ_preview)    │
│                        │  v_target = speed_target(effective_κ)    │
│                        │  v_cmd = rate_limit(v_target, dt)        │
│                        ▼                                         │
│                   ⑩  제어 명령 발행                                │
│                        │  → /t870/control_command                 │
│                        │  → /erp42/control_command (lazy)         │
│                        ▼                                         │
│                   ⑪  디버그 시각화 발행 (lazy)                     │
│                        │  → /pp_debug/lookahead_point (SPHERE)    │
│                        │  → /pp_debug/pursuit_arc (LINE_STRIP)    │
└──────────────────────────────────────────────────────────────────┘
```

---

## 내부 함수 상세

### `norm2d(x, y)` — 2D 유클리드 거리

```cpp
static double norm2d(double x, double y) { return sqrt(x*x + y*y); }
```
- 두 점 사이의 거리(누적 arc length) 또는 원점→목표점 직선 거리(Ld) 계산에 사용.

### `on_path(msg)` — 경로 수신 콜백

- `latest_points_` ← `msg->points` (덮어쓰기, 최신 경로만 유지)
- `last_path_time_` ← `now()` (타임아웃 판단용)

### `path_fresh()` — 경로 유효성 판단

- `latest_points_`가 비어있거나, 마지막 수신 후 `path_timeout_sec_` 초과 시 `false` 반환.
- planning 노드 장애나 LiDAR 끊김 감지 역할.

### `find_nearest_index()` — 최근접점 탐색

- 경로의 모든 점에 대해 원점(0,0)과의 거리² 계산.
- 가장 가까운 점의 인덱스를 반환.

### `compute_dynamic_lookahead(speed)` — 속도 적응형 lookahead

```
Ld = clamp(Ld_min + Ld_gain × speed, Ld_min, Ld_max)
```
- 속도가 빠르면 → 더 먼 곳을 주시 (안정적 추종)
- 속도가 느리면 → 가까운 곳을 주시 (민첩한 코너링)

### `compute_preview_curvature(nearest_i, preview_distance)` — 전방 곡률 미리보기

1. `nearest_i`부터 경로를 따라가며 `preview_distance`(2.5m) 내의 구간을 확인.
2. 연속 3점(a, b, c)에서 외적 기반 곡률을 계산: `κ = 2|cross| / (|ab|·|bc|·|ac|)`.
3. 구간 내 **최대 절대 곡률**을 반환.

### `compute_speed_target(abs_kappa)` — 곡률 기반 목표 속도

```
v = clamp(√(a_lat_limit / κ), v_min, v_max)
```
- 곡률이 0에 가까우면 → `v_max` (직선 최대 속도)
- 곡률이 크면 → 횡가속도 한계에 맞춘 감속

### `compute_target_relative(nearest_i, Ld, tx, ty, Ld_used)` — lookahead 목표점 탐색

1. `nearest_i`부터 경로를 따라가며 점 간 거리를 누적.
2. 누적 거리 ≥ `Ld`인 첫 번째 점을 목표로 선택.
3. 경로 끝까지 가도 부족하면 마지막 점 사용 (폴백).
4. `Ld_used`는 원점→목표점 **직선 거리** (Pure Pursuit 공식 요구).

### `rate_limit_speed(target, dt)` — 가감속 rate 제한

```
가속 시: v_cmd = min(v_target, v_prev + accel_rate × dt)
감속 시: v_cmd = max(v_target, v_prev - decel_rate × dt)
```
- 급격한 가감속 방지, 차량 안정성 확보.

### `publish_stop()` — 안전 정지 명령 발행

- `speed=0, steering=0` 발행 (T870 + ERP42).
- `last_cmd_speed_` 초기화.

---

## Pure Pursuit 알고리즘 상세

### 기본 원리

차량 전방의 "목표점(lookahead point)"을 향해 원호(circular arc)를 그리며 이동하도록 조향하는 기하학적 경로 추종 방법.

### 수식

```
곡률:    κ = 2 · y_target / Ld²
조향각:  δ = atan(L · κ)
```

- `y_target`: 목표점의 차량 기준 횡방향 거리 (좌: +, 우: -)
- `Ld`: 차량 원점에서 목표점까지의 직선 거리
- `L`: 차량 축간거리 (wheelbase, 0.87m)

### 기하학적 유도

```
  ──────────────────────────────────────────────────────────
  [기본 좌표계]

        목표점 T = (tx, ty)
           *
          /|
    Ld  /  |  ty (횡방향)
       /   |
      / α  |
     *-----+
   차량    tx (종방향)
   (0,0)

   - 차량은 원점 (0, 0), 전방이 +x
   - Ld = sqrt(tx² + ty²)  ... 차량→목표점 직선거리
   - α  = atan2(ty, tx)     ... 목표점 방위각

  ──────────────────────────────────────────────────────────
  [Step 1] 원호 모델 — 왜 원호인가?

   Pure Pursuit는 "차량이 일정한 조향각으로 주행하면
   원호를 그린다"는 사실을 이용한다.

   차량(0,0)과 목표점 T를 동시에 지나는 원을 찾으면,
   그 원의 반지름 R이 곧 필요한 회전 반지름이 된다.

         원의 중심 O = (0, R)
              |
              |  R
              |
     차량 *---+        ← 원의 중심은 항상 차량의 좌측(y+) 또는 우측(y-)에 있음
     (0,0)

   원의 중심을 O = (0, R)로 놓으면 (좌회전 가정):
     |O - 차량|  = R        ← 당연히 성립
     |O - T|    = R        ← 목표점도 같은 원 위

  ──────────────────────────────────────────────────────────
  [Step 2] 반지름 R 유도

   |O - T|² = R² 조건을 전개:
     (tx - 0)² + (ty - R)² = R²
     tx² + ty² - 2·ty·R + R² = R²
     tx² + ty² - 2·ty·R = 0

   Ld² = tx² + ty² 이므로:
     Ld² = 2·ty·R

   따라서:
     R = Ld² / (2·ty)

  ──────────────────────────────────────────────────────────
  [Step 3] 곡률 κ (curvature)

   곡률은 반지름의 역수:
     κ = 1/R = 2·ty / Ld²

   - ty > 0 → κ > 0 → 좌회전
   - ty < 0 → κ < 0 → 우회전
   - ty = 0 → κ = 0 → 직진

  ──────────────────────────────────────────────────────────
  [Step 4] 조향각 δ (Ackermann 기하학)

   자전거 모델(bicycle model)에서:

       ┌──── 앞바퀴 (조향)
       │  δ ↙ (조향각)
       │ /
       │/
       L  (wheelbase, 축간거리)
       │
       │
       └──── 뒷바퀴 (고정)

   회전 반지름과 조향각의 관계:
     tan(δ) = L / R

   따라서:
     δ = atan(L / R)
       = atan(L · κ)            ← κ = 1/R 대입
       = atan(L · 2·ty / Ld²)   ← κ 전개

  ──────────────────────────────────────────────────────────
  [최종 공식 요약]

     κ = 2·ty / Ld²           ... 목표점 횡방향 오프셋으로 곡률 결정
     δ = atan(L · κ)           ... 곡률에 축간거리를 곱해 조향각 산출
     δ = clamp(δ, -δ_max, +δ_max)  ... 하드웨어 한계로 클램핑

  ──────────────────────────────────────────────────────────
```

### Lookahead 목표점 선택 과정

1. **최근접점 탐색**: 경로의 모든 점 중 차량 원점(0,0)에 가장 가까운 점을 찾음
2. **누적 arc length**: 최근접점부터 경로를 따라가며 점 간 거리를 누적
3. **목표점 결정**: 누적 거리 ≥ lookahead 거리가 되는 첫 번째 점을 선택
4. **폴백**: 경로 끝까지 가도 lookahead 미달 시 마지막 점 사용

### 왜 "Relative" 버전인가?

일반 Pure Pursuit는 글로벌 좌표계에서 차량 위치/heading (GPS/odometry)이 필요하다.
이 노드는 planning이 이미 base_link 기준 상대좌표 경로를 생성하므로, GPS 없이 동작한다.
**대회 규정상 차선 구간에서 GPS 사용이 금지**되어 있으므로 이 방식이 필수적.

---

## 속도 제어 시스템

기존 고정 속도 방식에서 **곡률 기반 적응형 속도 제어**로 업그레이드되었다.

### 전체 속도 결정 흐름

```
                    전방 2.5m 구간
                    ┌────────────┐
경로 점들 ──→ compute_preview_curvature() ──→ κ_preview (전방 최대 곡률)
                                                │
                                                ▼
                                     compute_speed_target(κ_preview)
                                                │
                                                ▼
                                     v_preview = √(a_lat / κ_preview)
                                                │
                                                ▼
                                     compute_dynamic_lookahead(v_preview)
                                                │
                                                ▼
                                         Ld (적응형 lookahead)
                                                │
                    ┌───────────────────────────┘
                    ▼
        compute_target_relative(Ld) ──→ (tx, ty, Ld_used)
                    │
                    ▼
            κ_pp = 2·ty / Ld²  (Pure Pursuit 곡률)
                    │
                    ▼
            effective_κ = max(|κ_pp|, κ_preview)
                    │
                    ▼
            v_target = compute_speed_target(effective_κ)
                    │
                    ▼
            v_cmd = rate_limit_speed(v_target, dt)
                    │
                    ▼
            최종 속도 명령 발행
```

### Preview Curvature (전방 곡률 미리보기)

- `preview_distance` (2.5m) 범위 내의 경로에서 **3점 외적 기반 곡률**을 계산.
- 구간 내 최대 곡률을 반환하여 **선감속(코너 진입 전 미리 감속)**을 가능하게 함.

### Dynamic Lookahead (속도 연동 주시 거리)

```
Ld = clamp(Ld_min + Ld_gain × speed, Ld_min, Ld_max)
```

| 상황 | 속도 | Lookahead | 효과 |
|------|------|-----------|------|
| 급코너 | 느림 (0.45 m/s) | 짧음 (~0.85m) | 경로 밀착 추종 |
| 완만한 커브 | 중간 | 중간 | 균형 |
| 직선 | 빠름 (1.20 m/s) | 길음 (~1.70m) | 안정적 직진 |

### Rate Limiting (가감속 제한)

| 구분 | Rate | 설명 |
|------|------|------|
| 가속 | `accel_rate` (1.20 m/s²) | 직선 진입 시 급가속 방지 |
| 감속 | `decel_rate` (1.80 m/s²) | 코너 진입 시 빠르게 감속 (가속보다 빠름) |

감속 rate가 가속 rate보다 큰 이유: 안전을 위해 감속은 빠르게, 가속은 부드럽게.

---

## 파라미터

설정 파일: `config/pure_pursuit.yaml`

### 토픽 설정

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `path_topic` | string | `/planning/path` | 경로 입력 토픽 |
| `cmd_topic` | string | `/t870/control_command` | T870 제어 출력 토픽 |

### 차량 파라미터

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|----------|------|--------|------|------|
| `wheelbase` | double | 0.87 | m | T870 축간거리 (Ackermann 기하학 핵심) |
| `delta_max` | double | 0.314 | rad | 최대 조향각 (~18°, 하드웨어 한계) |

### Lookahead 파라미터 (속도 적응형)

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|----------|------|--------|------|------|
| `lookahead` | double | 1.2 | m | legacy fallback (min/max 미설정 시 사용) |
| `lookahead_min` | double | 0.85 | m | 코너에서의 최소 주시 거리 |
| `lookahead_max` | double | 1.70 | m | 직선에서의 최대 주시 거리 |
| `lookahead_speed_gain` | double | 0.65 | m/(m/s) | 속도 1m/s 증가당 lookahead 증가량 |

### 속도 제어 파라미터

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|----------|------|--------|------|------|
| `speed` | double | 1.0 | m/s | legacy fallback (min/max 미설정 시 사용) |
| `speed_min` | double | 0.45 | m/s | 급코너 최소 속도 |
| `speed_max` | double | 1.20 | m/s | 직선 최대 속도 |
| `lateral_accel_limit` | double | 0.90 | m/s² | 곡률 기반 감속 횡가속도 한계 |
| `preview_distance` | double | 2.50 | m | 전방 curvature preview 거리 |
| `accel_rate` | double | 1.20 | m/s² | 직선 가속 rate limit |
| `decel_rate` | double | 1.80 | m/s² | 코너 진입 감속 rate limit |

### 안전 파라미터

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|----------|------|--------|------|------|
| `path_timeout_sec` | double | 0.5 | sec | 경로 타임아웃 (초과 시 정지) |
| `min_x_target` | double | 0.05 | m | 목표점 최소 전방 거리 (이하 시 정지) |

---

## 안전 메커니즘 (정지 조건)

| 조건 | 원인 | 위험 |
|------|------|------|
| `path_fresh() == false` | planning 노드 장애 / LiDAR 끊김 | 이전 경로로 무한정 주행 |
| planning status FAIL | `not enough seeds` / `no valid path` / `too short valid path` | 유효하지 않은 경로 추종 |
| `poses.size() < 2` | 경로 점 부족 | 방향 결정 불가 |
| `Ld_used < 1e-3` | 목표점이 차량 위에 있음 | 0 나누기 → 조향 발산 |
| `tx ≤ min_x_target` | 목표점이 뒤쪽/측면 | 180도 회전 등 비정상 동작 |

모든 정지 조건에서 T870: `speed=0, steering=0`, ERP42: `speed=0, steering=0, brake=1` 명령을 발행한다.

---

## 디버그 시각화

RViz2에서 확인 가능. **lazy publisher**로 구독자가 없으면 발행하지 않아 성능 부담 없음.

### Lookahead Point (`/pp_debug/lookahead_point`)
- **Marker 타입**: SPHERE (초록색 구)
- **위치**: (tx, ty, 0) in base_link
- **크기**: 지름 0.15m
- **의미**: PP가 선택한 lookahead 목표점

### Pursuit Arc (`/pp_debug/pursuit_arc`)
- **Marker 타입**: LINE_STRIP (노란색 선)
- **내용**: 약 30개 점으로 샘플링된 원호 또는 직선
- **동작 방식**:
  - `κ ≈ 0`: 원점 → 목표점 직선 보간
  - `κ ≠ 0`: 회전 중심 `(0, R=1/κ)` 기준 원호를 그림
- **의미**: 현재 조향으로 차량이 따라갈 예상 궤적

---

## 로깅

| 레벨 | Throttle | 내용 |
|------|----------|------|
| INFO | 시작 시 1회 | 전체 파라미터 덤프 (`path, cmd, L, Ld, v, a_lat, delta_max`) |
| INFO | 500ms | 제어 상태 (`target, Ld, κ_pp, κ_preview, v_target, v_cmd, delta`) |
| WARN | 1000ms | 경로 미수신/타임아웃, 목표점 계산 실패, 안전 위반 |

---

## 스레드 안전성

- ROS 2 **single-threaded executor** (기본) 사용.
- `on_path()` 콜백과 `on_timer()` 타이머는 동시에 실행되지 않음.
- `latest_points_`, `last_path_time_` 등 공유 변수에 대한 mutex 불필요.

---

## 실행 방법

```bash
# 빌드
cd ~/ev_ws && colcon build --symlink-install --packages-select pp_controller_cpp

# 런치 파일로 실행 (권장 — config/pure_pursuit.yaml 자동 로드)
ros2 launch pp_controller_cpp pure_pursuit.launch.py

# 직접 실행 (기본 파라미터)
ros2 run pp_controller_cpp pure_pursuit_relative_node

# 파라미터 오버라이드
ros2 run pp_controller_cpp pure_pursuit_relative_node \
  --ros-args \
  -p lookahead_min:=1.0 \
  -p lookahead_max:=2.0 \
  -p speed_max:=1.5 \
  -p delta_max:=0.5
```

---

## 튜닝 가이드

| 증상 | 조정 방향 |
|------|----------|
| 코너에서 안쪽으로 빠짐 (understeer) | `lookahead_min` ↓ 또는 `lookahead_speed_gain` ↓ |
| 경로 추종이 지그재그로 진동 | `lookahead_min` ↑ 또는 `lookahead_speed_gain` ↑ |
| 급커브에서 조향이 부족 | `delta_max` ↑ (하드웨어 한계 확인) |
| 코너 진입 속도가 너무 빠름 | `lateral_accel_limit` ↓ 또는 `preview_distance` ↑ |
| 코너 진입 감속이 너무 급격 | `decel_rate` ↓ |
| 직선에서 가속이 너무 느림 | `accel_rate` ↑ 또는 `speed_max` ↑ |
| 직선에서 미세한 좌우 떨림 | `lookahead_max` ↑ 또는 planning 측 smoothing 강화 |
| 차량이 자주 멈춤 | `path_timeout_sec` ↑ 또는 `min_x_target` ↓ |
| 전체적으로 속도가 너무 느림 | `speed_min` ↑, `speed_max` ↑, `lateral_accel_limit` ↑ |
