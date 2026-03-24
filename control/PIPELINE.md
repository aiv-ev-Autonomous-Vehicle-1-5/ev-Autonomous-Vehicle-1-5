# Control Package Pipeline (pp_controller_cpp)

## 개요

T870 자율주행 차량용 상대좌표 Pure Pursuit 경로추종 제어기.
ROS 2 Humble, C++17, Component architecture.

Planning 모듈이 생성한 base_link 기준 상대좌표 경로(Marker POINTS)를 입력받아,
기하학적 경로 추종 알고리즘(Pure Pursuit)으로 **속도 적응형 lookahead**, **곡률 기반 속도 제어**, **rate-limited 가감속**을 적용하여
조향각과 속도를 계산하고 T870 실차 및 ERP42 Gazebo 시뮬레이터에 전달한다.

- **Node**: `pure_pursuit_relative_node` (50Hz wall timer)

---

## 제어 파이프라인

```
on_timer() — 50Hz (20ms)
│
│  control_dt = clamp(now - last_control_time, 1ms, 200ms)
│  첫 콜백은 0.02s(50Hz) 기본값 사용
│
├── ⓪ CREEP ROI 디버그 마커 발행 (lazy)
│     구독자가 있을 때만 /pp_debug/creep_roi 발행
│     ROI 범위: 전방 (0 ~ roi_x) × (±roi_y) 직사각형
│
├── ① 정지 조건 판정 + Fail Counter
│     [조건 검사]
│       a) path_fresh() 실패 → 경로 미수신 또는 path_timeout_sec(0.5s) 초과
│       b) latest_status_ == "FAIL - not enough seeds"
│       c) latest_status_ == "FAIL - no valid path"
│       d) latest_status_ == "WARNING - too short valid path"
│       → 하나라도 해당 시 should_stop = true
│
│     [카운터 로직]
│       should_stop == true:
│         fail_counter_++ (1회 증가)
│         ├─ counter < emergency_stop_count(40) → 이전 명령 유지, return
│         │   (단발성 노이즈 필터링: 40회 × 20ms = 800ms 동안 마지막 정상 속도 유지)
│         │
│         └─ counter ≥ 40:
│             ├─ is_forward_clear() == true → CREEP 모드
│             │     전방 ROI (roi_x × roi_y) 내 bbox 없음
│             │     v_cmd = rate_limit(creep_speed, last_cmd, dt, accel, decel)
│             │     조향 0°로 직진, return
│             │
│             └─ is_forward_clear() == false → 긴급 감속
│                   publish_emergency_decel(dt)
│                   v_cmd = rate_limit(0.0, last_cmd, dt, accel, emergency_decel_rate)
│                   목표속도 0으로 매 콜백마다 emergency_decel_rate로 감속
│                   조향 0°, return
│
│       should_stop == false:
│         fail_counter_ = 0 (즉시 리셋)
│         → 이하 정상 파이프라인 진행
│
├── ② 최근접점 탐색 (find_nearest_index)
│     경로 전체 점에서 차량 원점(0,0)까지 유클리드 거리가 최소인 점의 인덱스 반환
│     nearest_i = argmin(sqrt(pts[i].x² + pts[i].y²))
│
├── ③ 전방 곡률 분석 (compute_preview_curvature)
│     nearest_i부터 경로를 따라 arc length 누적, preview_distance(2.5m)까지 구간 결정
│     구간 내 연속 3점(A,B,C)마다 Menger 곡률 계산:
│       kappa = 2 × |cross(AB, BC)| / (|AB| × |BC| × |AC|)
│     preview_kappa = 구간 내 최대 곡률
│
├── ④ 동적 Lookahead 거리 계산
│     [preview 기반 목표 속도]
│       preview_speed_target = sqrt(lateral_accel_limit / preview_kappa)
│       clamp(preview_speed_target, speed_min, speed_max)
│       → 전방에 급커브가 보이면 목표 속도↓
│
│     [동적 lookahead]
│       lookahead_cmd = clamp(
│         lookahead_min + speed_gain × preview_speed_target,
│         lookahead_min,
│         lookahead_max)
│       → 느려질수록 lookahead 짧게 = 커브 대응력 향상
│       → 빨라질수록 lookahead 길게 = 직선 안정성 향상
│           * 실제 차량 속도 명령에는 쓰이지 않습니다. 
            * 속도 결정(⑧단계)에서는 v_curvature와 v_path_end만 사용합니다.

즉 preview_speed_target의 역할은 **"전방 곡률이 크면 lookahead를 짧게 → 커브 대응력 향상"**이라는 간접적 속도 반영 용도입니다.
├── ⑤ 타겟 포인트 계산 (compute_target_relative)
│     nearest_i부터 경로를 따라 arc length 누적
│     누적 거리 ≥ lookahead_cmd 되는 첫 번째 점을 목표로 선택
│     경로 끝까지 못 찾으면 마지막 점을 fallback 사용
│     tx, ty = 목표점 좌표 (base_link 기준)
│     Ld_used = 차량 원점 → 목표점 직선 거리 (arc length 아님)
│
│     [실패 시] 목표점 계산 불가 → publish_emergency_decel(), return
│
├── ⑥ 안전 조건 확인
│     a) Ld_used < 1e-3 → 목표점이 차량 위에 있음 (0 나누기 방지)
│        → publish_emergency_decel(), return
│     b) tx ≤ min_x_target → 목표점이 차량 뒤쪽 (비정상)
│        → publish_emergency_decel(), return
│
├── ⑦ Pure Pursuit 조향각 계산
│     곡률:  kappa_pp = 2 × ty / Ld_used²
│       ty > 0 → kappa > 0 → 좌회전
│       ty < 0 → kappa < 0 → 우회전
│       ty ≈ 0 → kappa ≈ 0 → 직진
│
│     조향각: delta = atan(wheelbase × kappa_pp)
│             delta = clamp(delta, -delta_max, +delta_max)
│       delta_max = 0.3249 rad (≈18.6°) = 물리적 조향 한계
│
├── ⑧ 속도 결정 (3개 후보 중 최소값)
│     [a) 곡률 기반 목표 속도]
│       effective_kappa = max(|kappa_pp|, preview_kappa)
│         현재 조향 곡률과 전방 preview 곡률 중 큰 값 사용
│       v_curvature = sqrt(lateral_accel_limit(상수,not parameter) / effective_kappa)
│       clamp(v_curvature, speed_min, speed_max)
│
│     [b) 제동거리 기반 목표 속도]
│       raw_remaining = nearest_i부터 경로 끝까지 arc length 합산
│       filtered_remaining = compute_filtered_remaining(raw_remaining)
│         이동평균 윈도우 크기 = path_length_filter_size(20)
│         급락 감지: raw < avg × path_drop_ratio(0.4) 시 buffer 동결
│           - 의심 중: pending에 보류, avg 반환 (노이즈 무시)
│           - N프레임(path_drop_confirm_count=3) 연속 확정 → raw 즉시 반영
│           - 중간에 정상 복귀 → pending 전부 buffer에 반영, avg 반환
│       effective_remaining = max(0, filtered_remaining - stop_margin)
│         stop_margin(1.3m) 남기고 속도 0 도달 목표
│       v_path_end = sqrt(2 × decel_rate × effective_remaining) 
│       clamp(v_path_end, 0, speed_max)
│
│     [c) 최종 선택]
│       v_target = min(v_curvature, v_path_end)
│         v_curvature: 곡률이 클수록 낮아지는 횡가속도 제한 속도
│         v_path_end:  경로 끝까지 남은 거리로부터 역산한 정지 가능 속도
│
├── ⑨ Rate Limiting 적용 (rate_limit_speed)
│     v_prev = last_cmd_speed_ (직전 프레임에서 발행한 속도)
│     dt = 콜백 주기 (초)
│
│     가속: v_cmd = min(v_target, v_prev + accel_rate × dt)
│       v_target > v_prev 일 때 적용
│       한 프레임당 accel_rate×dt 만큼만 속도 증가 허용
│       min() → v_target을 초과하지 않도록 상한 제한
│
│     감속: v_cmd = max(v_target, v_prev - decel_rate × dt)
│       v_target < v_prev 일 때 적용
│       한 프레임당 decel_rate×dt 만큼만 속도 감소 허용
│       max() → v_target 아래로 내려가지 않도록 하한 제한
│
│     → 급격한 속도 변화 방지, 부드러운 가감속 보장
│     → v_target이 갑자기 바뀌어도 실제 명령은 매 프레임 일정 폭씩만 변화
│     last_cmd_speed_ = v_cmd (다음 콜백에서 v_prev로 사용)
│
├── ⑩ 제어 명령 발행
│     cmd_pub_.publish(v_cmd, delta)
│       /t870/control_command → T870 실차 (항상)
│       /erp42/control_command → ERP42 Gazebo (lazy)
│
└── ⑪ 디버그 (lazy — 구독자 있을 때만)
      [로그] 500ms마다 throttle 출력:
        target=(tx, ty), Ld, kappa_pp, kappa_prev,
        v_target, v_cmd, delta, remain, filtered, v_curv, v_end
      [시각화]
        /pp_debug/lookahead_point — 초록 SPHERE (목표점)
        /pp_debug/pursuit_arc — 노란 LINE_STRIP (예상 원호 궤적)
```

---

## 특수 기능

### CREEP 모드

전방 ROI 영역 내에 장애물이 없을 때(클리어) 저속 직진하는 모드.
- 전방 ROI: `creep_roi_x` × `creep_roi_y` (1.0m × 0.5m)
- CREEP 속도: `creep_speed` (0.4 m/s)
- `/perception/bboxes`를 구독하여 전방 장애물 존재 여부를 판단

### 비상감속 (Emergency Decel)

급정지 시 역전기력(back-EMF)에 의한 하드웨어 손상을 방지하기 위해 점진적 감속.
- `emergency_decel_rate` (2.0 m/s²)로 감속
- `emergency_stop_count` (40회) 연속 FAIL 시 발동

### Fail Counter

planning FAIL 및 경로 타임아웃의 단발성 노이즈를 필터링하는 연속 카운터.
- FAIL 1~39회: 이전 명령 유지 (노이즈 무시)
- FAIL 40회 연속: 긴급 감속 시작
- 정상 복귀 시: 카운터 즉시 리셋

---

## 패키지 구조

```
control/
├── CMakeLists.txt
├── package.xml                                 # pp_controller_cpp
├── PIPELINE.md
├── topic.md
├── config/
│   └── pure_pursuit.yaml
├── launch/
│   └── pure_pursuit.launch.py
├── include/pp_controller_cpp/
│   ├── common/
│   │   ├── geometry.hpp                        # 2D 기하학 유틸 (norm2d, header-only)
│   │   └── params.hpp                          # 파라미터 구조체 + load() (header-only)
│   ├── pursuit/
│   │   ├── path_query.hpp                      # 경로 조회 알고리즘
│   │   ├── speed_planning.hpp                  # 속도 계획 알고리즘
│   │   └── steering.hpp                        # 조향 계산 알고리즘
│   ├── debug/
│   │   └── debug_visualizer.hpp                # RViz2 디버그 시각화
│   └── nodes/
│       ├── pure_pursuit_relative_node.hpp      # 노드 클래스 선언
│       └── command_publisher.hpp               # 제어 명령 발행 헬퍼
└── src/
    ├── pursuit/
    │   ├── path_query.cpp                      # 최근접점, 타겟포인트 탐색
    │   ├── speed_planning.cpp                  # 곡률/제동거리 기반 속도 계산
    │   └── steering.cpp                        # Pure Pursuit 조향각 계산
    ├── debug/
    │   └── debug_visualizer.cpp                # RViz2 마커 생성/발행
    └── nodes/
        ├── pure_pursuit_relative_node.cpp      # 노드 오케스트레이터
        └── command_publisher.cpp               # T870/ERP42 명령 발행
```

### 모듈 구조

| 모듈 | 역할 | 비고 |
|------|------|------|
| `common/geometry.hpp` | `norm2d()` 2D 거리 유틸 | header-only, ROS 비의존 |
| `common/params.hpp` | `PurePursuitParams` 구조체 + `load()` | header-only, 파라미터 declare/get |
| `pursuit/path_query` | 경로 조회 (최근접점, 타겟포인트) | ROS 비의존 |
| `pursuit/speed_planning` | 곡률/제동거리 기반 속도 계산 | ROS 비의존 |
| `pursuit/steering` | Pure Pursuit 조향각 계산 | ROS 비의존 |
| `debug/debug_visualizer` | RViz2 마커 생성/발행 (lazy) | 알고리즘 비의존 |
| `nodes/pure_pursuit_relative_node` | 노드 오케스트레이터 | 모듈 조합, 제어 루프 |
| `nodes/command_publisher` | T870/ERP42 제어 명령 발행 | 명령 포맷 캡슐화 |

---

## 빌드 타겟

| 타겟 | 타입 | 설명 |
|------|------|------|
| `pp_pursuit` | 공유 라이브러리 | pursuit/ 알고리즘 (path_query, speed_planning, steering) |
| `pp_debug_viz` | 공유 라이브러리 | debug/ RViz2 시각화 |
| `pp_controller_component` | ComposableNode | nodes/ 노드 오케스트레이터 + command_publisher |

### 빌드 명령

```bash
cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select pp_controller_cpp
```

---

## 파라미터

설정 파일: `config/pure_pursuit.yaml`

### 차량 파라미터

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `wheelbase` | 0.87 | m | T870 축간거리 |
| `delta_max` | 0.314 | rad | 최대 조향각 (~18°) |

### Lookahead 파라미터

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `lookahead_min` | 0.85 | m | 코너 최소 주시 거리 |
| `lookahead_max` | 1.50 | m | 직선 최대 주시 거리 |
| `lookahead_speed_gain` | 0.35 | m/(m/s) | 속도 증가당 lookahead 증가량 |

### 속도 제어 파라미터

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `speed_min` | 0.2 | m/s | 급코너 최소 속도 |
| `speed_max` | 0.7 | m/s | 직선 최대 속도 |
| `lateral_accel_limit` | 0.90 | m/s² | 곡률 기반 감속 횡가속도 한계 |
| `preview_distance` | 2.50 | m | 전방 curvature preview 거리 |
| `accel_rate` | 0.20 | m/s² | 직선 가속 rate limit |
| `decel_rate` | 3.00 | m/s² | 코너 진입 감속 rate limit |

### 안전 파라미터

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `emergency_decel_rate` | 2.0 | m/s² | 비상 감속 rate |
| `emergency_stop_count` | 40 | 회 | 연속 FAIL 허용 횟수 |
| `path_timeout_sec` | 0.5 | sec | 경로 타임아웃 |
| `stop_margin` | 1.3 | m | 정지 마진 |

### CREEP 모드 파라미터

| 파라미터 | 값 | 단위 | 설명 |
|----------|-----|------|------|
| `creep_speed` | 0.4 | m/s | CREEP 모드 속도 |
| `creep_roi_x` | 1.0 | m | 전방 ROI X 범위 |
| `creep_roi_y` | 0.5 | m | 전방 ROI Y 범위 |

---

## Pure Pursuit 알고리즘 상세

### 기본 원리

차량 전방의 "목표점(lookahead point)"을 향해 원호(circular arc)를 그리며 이동하도록 조향하는 기하학적 경로 추종 방법.

### 수식

```
곡률:    κ = 2 · y_target / Ld²
조향각:  δ = atan(L · κ)
         δ = clamp(δ, -δ_max, +δ_max)
```

- `y_target`: 목표점의 차량 기준 횡방향 거리 (좌: +, 우: -)
- `Ld`: 차량 원점에서 목표점까지의 직선 거리
- `L`: 차량 축간거리 (wheelbase, 0.87m)

### 왜 "Relative" 버전인가?

일반 Pure Pursuit는 글로벌 좌표계에서 차량 위치/heading (GPS/odometry)이 필요하다.
이 노드는 planning이 이미 base_link 기준 상대좌표 경로를 생성하므로, GPS 없이 동작한다.
**대회 규정상 차선 구간에서 GPS 사용이 금지**되어 있으므로 이 방식이 필수적.

---

## 속도 제어 시스템

### 전체 속도 결정 흐름

```
경로 점들 → compute_preview_curvature() → κ_preview (전방 최대 곡률)
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
                                            ▼
                        compute_target_relative(Ld) → (tx, ty, Ld_used)
                                            │
                                            ▼
                                 κ_pp = 2·ty / Ld²
                                            │
                                            ▼
                                 effective_κ = max(|κ_pp|, κ_preview)
                                            │
                                            ▼
                                 v_curv = compute_speed_target(effective_κ)
                                 v_end  = √(2 × decel × remaining_len)
                                 v_target = min(v_curv, v_end)
                                            │
                                            ▼
                                 v_cmd = rate_limit_speed(v_target, dt)
```

---

## 안전 메커니즘 (정지 조건)

| 조건 | 원인 | 위험 |
|------|------|------|
| `path_fresh() == false` | planning 노드 장애 / LiDAR 끊김 | 이전 경로로 무한정 주행 |
| planning status FAIL | `not enough seeds` / `no valid path` / `too short valid path` | 유효하지 않은 경로 추종 |
| `poses.size() < 2` | 경로 점 부족 | 방향 결정 불가 |
| `Ld_used < 1e-3` | 목표점이 차량 위에 있음 | 0 나누기 → 조향 발산 |
| `tx ≤ min_x_target` | 목표점이 뒤쪽/측면 | 비정상 동작 |

모든 정지 조건에서 T870: `speed=0, steering=0`, ERP42: `speed=0, steering=0, brake=1` 명령을 발행한다.

---

## 디버그 시각화

RViz2에서 확인 가능. **lazy publisher**로 구독자가 없으면 발행하지 않아 성능 부담 없음.

| 토픽 | 마커 타입 | 색상 | 의미 |
|------|-----------|------|------|
| `/pp_debug/lookahead_point` | SPHERE | 초록 | PP가 선택한 lookahead 목표점 |
| `/pp_debug/pursuit_arc` | LINE_STRIP | 노란 | PP 곡률로부터 계산한 예상 원호 궤적 |
| `/pp_debug/creep_roi` | LINE_STRIP | 노란 | CREEP 모드 전방 ROI 영역 박스 |

---

## 실행 방법

```bash
# 빌드
cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select pp_controller_cpp

# 런치 파일로 실행 (권장)
ros2 launch pp_controller_cpp pure_pursuit.launch.py

# 직접 실행
ros2 run pp_controller_cpp pure_pursuit_relative_node
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
| 차량이 자주 멈춤 | `path_timeout_sec` ↑ 또는 `emergency_stop_count` ↑ |
| CREEP 모드가 너무 빠름 | `creep_speed` ↓ |
| CREEP ROI가 너무 좁음/넓음 | `creep_roi_x` / `creep_roi_y` 조정 |
