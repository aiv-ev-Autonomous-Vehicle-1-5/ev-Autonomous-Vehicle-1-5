# t870_rqt_plugin 패키지 토픽 정리

## ControlPanelPlugin

### 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | Reliable, depth=1 | 수동 차량 제어 명령 |

### 서비스 (클라이언트)

| 서비스 이름 | 서비스 타입 | 설명 |
|---|---|---|
| `/t870/mode_command` | `t870_msgs/srv/ModeCommand` | 모드/E-Stop/기어 설정 요청 |

---

## FeedbackMonitorPlugin

### 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/feedback` | `t870_msgs/msg/Feedback` | Reliable, depth=1 | 차량 상태 피드백 모니터링 |
