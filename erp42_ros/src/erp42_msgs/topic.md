# erp42_msgs 패키지 토픽 정리

이 패키지는 메시지/서비스 정의만 포함하며, ROS 토픽을 직접 구독/발행하지 않습니다.

## 정의된 메시지 타입

| 메시지 타입 | 필드 | 설명 |
|---|---|---|
| `ControlCommand` | `float64 speed`, `float64 steering`, `uint8 brake` | 차량 제어 명령 |
| `Feedback` | `header`, `manual_mode`, `emergency_stop`, `gear`, `speed`, `steering`, `brake`, `encoder_count`, `heartbeat` | 차량 상태 피드백 |

## 정의된 서비스 타입

| 서비스 타입 | Request | Response | 설명 |
|---|---|---|---|
| `ModeCommand` | `manual_mode`, `emergency_stop`, `gear` | `success` | 모드/E-Stop/기어 설정 |

## 사용하는 토픽

| 토픽 이름 | 메시지 타입 | 발행 노드 | 구독 노드 |
|---|---|---|---|
| `/erp42/control_command` | `ControlCommand` | pure_pursuit (Gazebo), rqt_control_panel | erp42_serial_bridge |
| `/erp42/feedback` | `Feedback` | erp42_serial_bridge | erp42_rqt_feedback_monitor |
| `/erp42/mode_command` | `ModeCommand` (srv) | serial_bridge (server) | launch (client) |
