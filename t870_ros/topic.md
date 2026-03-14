# t870_ros Topic 정리

## 노드 구성

| 노드 | 패키지 | 설명 |
|------|--------|------|
| `serial_bridge` | `t870_serial` | T870 PCU 시리얼 통신 브릿지 |
| `t870_control_panel` | `t870_rqt_plugin` | RQT 수동 제어 GUI |
| `t870_feedback_monitor` | `t870_rqt_plugin` | RQT 피드백 모니터 GUI |

---

## Topics

### `/t870/control_command`

| 항목 | 내용 |
|------|------|
| **타입** | `t870_msgs/msg/ControlCommand` |
| **Publisher** | `t870_control_panel` (RQT GUI, 50 Hz) |
| **Subscriber** | `serial_bridge` |
| **QoS** | Keep Last(1), Best Effort, Volatile |

**필드:**
| 필드 | 타입 | 설명 |
|------|------|------|
| `speed` | `float64` | 목표 속도 (m/s) |
| `steering` | `float64` | 목표 조향각 (rad) |

---

### `/t870/feedback`

| 항목 | 내용 |
|------|------|
| **타입** | `t870_msgs/msg/Feedback` |
| **Publisher** | `serial_bridge` |
| **Subscriber** | `t870_feedback_monitor` (RQT GUI) |
| **QoS** | Keep Last(1), Reliable, Volatile |

**필드:**
| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 |
| `manual_mode` | `bool` | 제어 모드 (true=수동, false=자율) |
| `emergency_stop` | `bool` | 비상 정지 상태 |
| `gear` | `uint8` | 기어 (0=전진, 1=중립, 2=후진) |
| `speed` | `float64` | 현재 속도 (m/s) |
| `steering` | `float64` | 현재 조향각 (rad) |
| `heartbeat` | `uint8` | 헬스체크 카운터 (0-255) |

---

## Service

### `/t870/mode_command`

| 항목 | 내용 |
|------|------|
| **타입** | `t870_msgs/srv/ModeCommand` |
| **Server** | `serial_bridge` |
| **Client** | `t870_control_panel` (RQT GUI) |

**Request:**
| 필드 | 타입 | 설명 |
|------|------|------|
| `manual_mode` | `bool` | 제어 모드 설정 (false=자율, true=수동) |
| `emergency_stop` | `bool` | 비상 정지 설정 |
| `gear` | `uint8` | 기어 설정 (0=전진, 1=중립, 2=후진) |

**Response:**
| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 처리 성공 여부 |

---

## Parameters (`serial_bridge`)

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `port_path` | string | `/dev/ttyUSB0` | 시리얼 포트 경로 |
| `baud_rate` | int | `115200` | 시리얼 통신 속도 |
| `max_speed_mps` | double | `1.60` | 최대 속도 (m/s) |
| `max_steering_deg` | double | `18.00` | 최대 조향각 (deg) |
| `steering_offset_deg` | double | `0.00` | 조향 오프셋 (deg) |

---

## 데이터 흐름

```
[제어 경로]
  RQT GUI (ControlPanel)
    └─ /t870/control_command ─→ serial_bridge ─→ T870 PCU (시리얼)

[피드백 경로]
  T870 PCU (시리얼) ─→ serial_bridge
    └─ /t870/feedback ─→ RQT GUI (FeedbackMonitor)

[모드 설정]
  RQT GUI (ControlPanel)
    └─ /t870/mode_command (srv) ─→ serial_bridge ─→ T870 PCU (시리얼)
```
