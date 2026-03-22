# t870_msgs 패키지 토픽 정리

이 패키지는 메시지/서비스 정의만 포함하며, ROS 토픽을 직접 구독/발행하지 않습니다.

## 정의된 메시지 타입

| 메시지 타입 | 필드 | 설명 |
|---|---|---|
| `ControlCommand` | `float64 speed`, `float64 steering` | 차량 제어 명령 (m/s, rad) |
| `Feedback` | `header`, `manual_mode`, `emergency_stop`, `gear`, `speed`, `steering`, `heartbeat` | 차량 상태 피드백 |

## 정의된 서비스 타입

| 서비스 타입 | Request | Response | 설명 |
|---|---|---|---|
| `ModeCommand` | `manual_mode`, `emergency_stop`, `gear` | `success` | 모드/E-Stop/기어 설정 |

## 사용하는 토픽

| 토픽 이름 | 메시지 타입 | 발행 노드 | 구독 노드 |
|---|---|---|---|
| `/t870/control_command` | `ControlCommand` | pure_pursuit_relative_node | serial_bridge, bbox_tracker |
| `/t870/feedback` | `Feedback` | serial_bridge | rqt_feedback_monitor |
| `/t870/mode_command` | `ModeCommand` (srv) | serial_bridge (server) | serial_bridge launch (client) |
