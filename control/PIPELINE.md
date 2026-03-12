# Control Package Pipeline (pp_controller_cpp)

## 개요

ERP42 차량을 위한 **Pure Pursuit 경로 추종 제어 노드**.
Planning 모듈이 생성한 base_link 기준 상대좌표 경로를 입력받아,
기하학적 경로 추종 알고리즘(Pure Pursuit)으로 조향각과 속도를 계산하여 차량에 전달한다.

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
  → /planning/path (Path)                    ← 이 노드의 입력
  → PurePursuitRelativeNode (조향 계산)       ← 이 노드
  → /erp42/control_command (ControlCommand)  ← 이 노드의 출력
  → ERP42 차량 인터페이스 (erp42_ros)
```

---

## 토픽 인터페이스

### 입력 (Subscribe)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/planning/path` | `nav_msgs/msg/Path` | depth=10 | base_link 기준 상대좌표 경로. 각 `PoseStamped.pose.position`의 x는 전방, y는 좌측이 양수. |

#### Path 메시지 상세

```
nav_msgs/msg/Path
├── header
│   ├── stamp        # 경로 생성 시각
│   └── frame_id     # "base_link" (상대좌표)
└── poses[]          # PoseStamped 배열
    └── pose
        └── position
            ├── x    # 차량 전방 거리 [m] (앞이 +)
            ├── y    # 차량 횡방향 거리 [m] (좌가 +)
            └── z    # 미사용 (0)
```

### 출력 (Publish)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | depth=10 | ERP42 차량 제어 명령 |

#### ControlCommand 메시지 상세

```
erp42_msgs/msg/ControlCommand
├── speed     # float64: 목표 속도 [m/s]
├── steering  # float64: 조향각 [rad] (좌회전 +, 우회전 -)
└── brake     # uint8:   브레이크 [0~150]
```

---

## 노드 내부 파이프라인

```
┌─────────────────────────────────────────────────────────┐
│                PurePursuitRelativeNode                   │
│                                                         │
│  /planning/path ──→ [on_path 콜백]                      │
│                        │                                │
│                        ▼                                │
│                   latest_path_ 저장                      │
│                   last_path_time_ 기록                   │
│                                                         │
│  20Hz 타이머 ──→ [on_timer 제어루프]                      │
│                        │                                │
│                   ①  path_fresh() 확인                   │
│                        │  (경로 비어있거나 timeout → 정지) │
│                        ▼                                │
│                   ②  compute_target_relative()           │
│                        │  - 최근접점 탐색                 │
│                        │  - 누적 arc length로 lookahead  │
│                        │    목표점 선택                   │
│                        ▼                                │
│                   ③  안전 조건 확인                       │
│                        │  - Ld < 1e-3 → 정지             │
│                        │  - tx ≤ min_x_target → 정지     │
│                        ▼                                │
│                   ④  Pure Pursuit 계산                   │
│                        │  kappa = 2*y / Ld²              │
│                        │  delta = atan(L * kappa)        │
│                        │  delta = clamp(delta, ±max)     │
│                        ▼                                │
│                   ⑤  ControlCommand 발행                 │
│                        │                                │
│                        ▼                                │
│              /erp42/control_command ──→                  │
└─────────────────────────────────────────────────────────┘
```

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
- `L`: 차량 축간거리 (wheelbase, 0.74m)

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

## 파라미터

| 파라미터 | 타입 | 기본값 | 단위 | 설명 |
|----------|------|--------|------|------|
| `path_topic` | string | `/planning/path` | - | 경로 입력 토픽 |
| `cmd_topic` | string | `/erp42/control_command` | - | 제어 출력 토픽 |
| `wheelbase` | double | 0.74 | m | ERP42 축간거리 |
| `lookahead` | double | 1.2 | m | 전방 주시 거리 |
| `speed` | double | 0.3 | m/s | 목표 주행 속도 |
| `delta_max` | double | 0.314 | rad | 최대 조향각 (≈18°) |
| `brake_stop` | int | 30 | 0~150 | 정지 시 브레이크 |
| `brake_run` | int | 0 | 0~150 | 주행 시 브레이크 |
| `path_timeout_sec` | double | 0.5 | sec | 경로 타임아웃 |
| `min_x_target` | double | 0.05 | m | 목표점 최소 전방 거리 |

---

## 안전 메커니즘 (정지 조건)

| 조건 | 원인 | 위험 |
|------|------|------|
| `path_fresh() == false` | planning 노드 장애 / LiDAR 끊김 | 이전 경로로 무한정 주행 |
| `poses.size() < 2` | 경로 점 부족 | 방향 결정 불가 |
| `Ld_used < 1e-3` | 목표점이 차량 위에 있음 | 0 나누기 → 조향 발산 |
| `tx ≤ min_x_target` | 목표점이 뒤쪽/측면 | 180도 회전 등 비정상 동작 |

모든 정지 조건에서 `speed=0, steering=0, brake=brake_stop_` 명령을 발행한다.

---

## 실행 방법

```bash
# 빌드
cd ~/ev_ws && colcon build --symlink-install --packages-select pp_controller_cpp

# 실행 (기본 파라미터)
ros2 run pp_controller_cpp pure_pursuit_relative_node

# 파라미터 오버라이드
ros2 run pp_controller_cpp pure_pursuit_relative_node \
  --ros-args \
  -p lookahead:=2.0 \
  -p speed:=0.5 \
  -p delta_max:=0.5
```

---

## 튜닝 가이드

| 증상 | 조정 방향 |
|------|----------|
| 경로를 부드럽게 따라가지만 코너에서 안쪽으로 빠짐 | `lookahead` ↓ |
| 경로 추종이 지그재그로 진동 | `lookahead` ↑ |
| 급커브에서 조향이 부족 | `delta_max` ↑ (하드웨어 한계 확인) |
| 차량이 자주 멈춤 | `path_timeout_sec` ↑ 또는 `min_x_target` ↓ |
| 직선에서 미세한 좌우 떨림 | `lookahead` ↑ 또는 planning 측 smoothing 강화 |
