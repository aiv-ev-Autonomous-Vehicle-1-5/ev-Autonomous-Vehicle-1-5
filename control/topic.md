# Control Package (pp_controller_cpp) — Topic 정리

## 입력 토픽 (Subscribe)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | depth=10 | base_link 기준 상대좌표 경로. 각 Point의 x는 전방(+), y는 좌측(+). |

## 출력 토픽 (Publish)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | depth=10 | ERP42 제어 명령 (speed, steering, brake) |

---

## 노드 정보

- **노드 이름**: `pure_pursuit_relative_node`
- **실행 주기**: 20Hz (50ms wall timer)
- **알고리즘**: Pure Pursuit (상대좌표 버전)

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `path_topic` | string | `/planning/path` | 경로 입력 토픽 |
| `cmd_topic` | string | `/erp42/control_command` | 제어 출력 토픽 |
| `wheelbase` | double | 0.74 | 축간거리 [m] |
| `lookahead` | double | 1.2 | 전방 주시 거리 [m] |
| `speed` | double | 0.3 | 목표 주행 속도 [m/s] |
| `delta_max` | double | 0.314 | 최대 조향각 [rad] (~18도) |
| `brake_stop` | int | 30 | 정지 시 브레이크 값 [0~150] |
| `brake_run` | int | 0 | 주행 시 브레이크 값 [0~150] |
| `path_timeout` | double | 0.5 | 경로 타임아웃 [초] |
| `min_x_target` | double | 0.05 | 목표점 최소 전방 거리 [m] |

## 정지 조건

- 경로 미수신 또는 타임아웃 (`path_timeout` 초과)
- 경로 점이 2개 미만
- 목표점까지 거리가 0에 가까움 (Ld < 1e-3)
- 목표점이 차량 뒤쪽 (tx <= `min_x_target`)
