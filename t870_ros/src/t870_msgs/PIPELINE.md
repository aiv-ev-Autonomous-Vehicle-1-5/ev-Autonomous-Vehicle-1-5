# t870_msgs 패키지 파이프라인

## 패키지 정보
- **Package**: t870_msgs
- **Description**: Henes T870 ROS2 메시지/서비스 인터페이스 정의

---

## 메시지

### 1. ControlCommand.msg

| 필드 | 타입 | 단위 | 설명 |
|------|------|------|------|
| `speed` | float64 | m/s | 목표 속도 |
| `steering` | float64 | rad | 목표 조향각 (+좌/-우) |

### 2. Feedback.msg

| 필드 | 타입 | 단위 | 설명 |
|------|------|------|------|
| `header` | std_msgs/Header | - | 타임스탬프 |
| `manual_mode` | bool | - | 제어 모드 (true=수동, false=자율) |
| `emergency_stop` | bool | - | 비상 정지 상태 |
| `gear` | uint8 | - | 기어 (0=전진, 1=중립, 2=후진) |
| `speed` | float64 | m/s | 현재 속도 |
| `steering` | float64 | rad | 현재 조향각 |
| `heartbeat` | uint8 | - | 헬스체크 카운터 (0-255) |

---

## 서비스

### 1. ModeCommand.srv

**Request:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `manual_mode` | bool | 제어 모드 설정 (true=수동, false=자율) |
| `emergency_stop` | bool | 비상 정지 설정 |
| `gear` | uint8 | 기어 설정 (0=전진, 1=중립, 2=후진) |

**Response:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | bool | 처리 성공 여부 |
