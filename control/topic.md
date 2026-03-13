# Control Package (pp_controller_cpp) — Topic 정리

## 입력 토픽 (Subscribe)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | BestEffort, depth=10 | base_link 기준 상대좌표 경로. 각 Point의 x는 전방(+), y는 좌측(+). |
| `/planning/status` | `std_msgs/msg/String` | BestEffort, depth=10 | planning 상태. "OK"=정상, "FAIL - ..."=실패 시 정지. |

## 출력 토픽 (Publish)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | BestEffort, depth=1 | T870 실차 제어 명령 (speed, steering) — 항상 발행 |
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | depth=10 | ERP42 Gazebo 시뮬레이션 제어 명령 (speed, steering, brake) — lazy |

## 디버그 토픽 (Publish, lazy — 구독자가 있을 때만 발행)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/pp_debug/lookahead_point` | `visualization_msgs/msg/Marker` (SPHERE) | BestEffort, depth=1 | PP가 선택한 lookahead 목표점. base_link 기준 (tx, ty, 0). 초록색 구. |
| `/pp_debug/pursuit_arc` | `visualization_msgs/msg/Marker` (LINE_STRIP) | BestEffort, depth=1 | PP 곡률(kappa)로부터 계산한 차량 예상 원호 궤적. 노란색 선. kappa≈0이면 직선. |

---

## 노드 정보

- **노드 이름**: `pure_pursuit_relative_node`
- **실행 주기**: 20Hz (50ms wall timer)
- **알고리즘**: Pure Pursuit (상대좌표 버전) + 곡률 기반 속도 제어

## 파라미터

### 토픽 설정

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `path_topic` | string | `/planning/path` | 경로 입력 토픽 |
| `cmd_topic` | string | `/t870/control_command` | T870 제어 출력 토픽 |

### 차량 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `wheelbase` | double | 0.87 | T870 축간거리 [m] |
| `delta_max` | double | 0.314 | 최대 조향각 [rad] (~18도) |

### Lookahead 파라미터 (속도 적응형)

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `lookahead` | double | 1.2 | legacy fallback 주시 거리 [m] |
| `lookahead_min` | double | 0.85 | 코너 최소 주시 거리 [m] |
| `lookahead_max` | double | 1.70 | 직선 최대 주시 거리 [m] |
| `lookahead_speed_gain` | double | 0.65 | 속도 증가당 lookahead 증가량 [m/(m/s)] |

### 속도 제어 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `speed` | double | 1.0 | legacy fallback 속도 [m/s] |
| `speed_min` | double | 0.45 | 급코너 최소 속도 [m/s] |
| `speed_max` | double | 1.20 | 직선 최대 속도 [m/s] |
| `lateral_accel_limit` | double | 0.90 | 곡률 기반 감속 횡가속도 한계 [m/s²] |
| `preview_distance` | double | 2.50 | 전방 curvature preview 거리 [m] |
| `accel_rate` | double | 1.20 | 직선 가속 rate limit [m/s²] |
| `decel_rate` | double | 1.80 | 코너 진입 감속 rate limit [m/s²] |

### 안전 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `path_timeout_sec` | double | 0.5 | 경로 타임아웃 [초] |
| `min_x_target` | double | 0.05 | 목표점 최소 전방 거리 [m] |

## 정지 조건

- 경로 미수신 또는 타임아웃 (`path_timeout_sec` 초과)
- planning 상태 FAIL (`"FAIL - not enough seeds"` / `"FAIL - no valid path"` / `"FAIL - too short valid path"`)
- 경로 점이 2개 미만
- 목표점까지 거리가 0에 가까움 (Ld < 1e-3)
- 목표점이 차량 뒤쪽 (tx <= `min_x_target`)
