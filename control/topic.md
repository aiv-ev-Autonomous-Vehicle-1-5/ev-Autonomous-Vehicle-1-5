# control 패키지 토픽 정리

## 구독 토픽 (Subscriptions)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/planning/path` | `visualization_msgs/msg/Marker` (POINTS) | BestEffort, depth=10 | 경로 포인트 (base_link 상대좌표) |
| `/planning/status` | `std_msgs/msg/String` | BestEffort, depth=10 | 플래닝 상태 ("OK", "FAIL - ...", "WARNING - ...") |
| `/perception/bboxes` | `ev_msgs/msg/BBoxArray` | BestEffort, depth=10 | CREEP 모드용 전방 장애물 감지 |

## 발행 토픽 (Publications)

### 제어 명령

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | BestEffort, depth=1 | T870 실차 제어 (속도, 조향) |
| `/erp42/control_command` | `erp42_msgs/msg/ControlCommand` | Reliable, depth=10 | Gazebo ERP42 시뮬레이터 제어 (Lazy) |

### 디버그 토픽 (Lazy)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/pp_debug/lookahead_point` | `visualization_msgs/msg/Marker` (SPHERE) | BestEffort, depth=1 | Lookahead 타겟 포인트 (초록 구) |
| `/pp_debug/pursuit_arc` | `visualization_msgs/msg/Marker` (LINE_STRIP) | BestEffort, depth=1 | 예측 호 궤적 (노란 선) |
| `/pp_debug/creep_roi` | `visualization_msgs/msg/Marker` (LINE_STRIP) | BestEffort, depth=1 | CREEP 모드 전방 ROI 영역 (노란 박스) |
