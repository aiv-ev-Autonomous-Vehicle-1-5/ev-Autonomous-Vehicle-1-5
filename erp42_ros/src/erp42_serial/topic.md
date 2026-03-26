# erp42_serial 패키지 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | Reliable, depth=1 | 차량 제어 명령 (header, speed, steering, brake). header는 센서 타임스탬프 전파용 |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/erp42/feedback` | `erp42_msgs/msg/Feedback` | Reliable, depth=1 | 차량 피드백 (모드, E-Stop, 기어, 속도, 조향, 브레이크, 엔코더, 하트비트) |
| `/erp42/odometry_wheel` | `nav_msgs/msg/Odometry` | depth=10 | 휠 오도메트리 (자전거 모델 기반) |

## 서비스 (서버)

| 서비스 이름 | 서비스 타입 | 설명 |
|---|---|---|
| `/erp42/mode_command` | `erp42_msgs/srv/ModeCommand` | 제어모드/E-Stop/기어 설정 |
