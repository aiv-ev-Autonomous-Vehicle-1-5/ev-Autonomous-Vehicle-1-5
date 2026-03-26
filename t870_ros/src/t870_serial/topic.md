# t870_serial 패키지 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | BestEffort, depth=1 | 차량 제어 명령 (header, speed m/s, steering rad). header는 센서 타임스탬프 전파용 |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/feedback` | `t870_msgs/msg/Feedback` | Reliable, depth=1 | 차량 피드백 (모드, E-Stop, 기어, 속도, 조향, 하트비트) |

## 서비스 (서버)

| 서비스 이름 | 서비스 타입 | 설명 |
|---|---|---|
| `/t870/mode_command` | `t870_msgs/srv/ModeCommand` | 제어모드/E-Stop/기어 설정 |
