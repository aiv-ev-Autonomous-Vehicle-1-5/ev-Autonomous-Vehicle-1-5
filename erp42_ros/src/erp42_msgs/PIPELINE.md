# erp42_msgs

## Package 정보
- **패키지명**: erp42_msgs
- **설명**: ERP42 ROS2 메시지/서비스 인터페이스 정의

## 메시지 정의

### 1. ControlCommand.msg
차량 제어 명령 메시지

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `speed` | `float64` | m/s | 차량 선속도 (음수 불가, 후진시 기어 변경 필요) |
| `steering` | `float64` | rad | 조향각 (+좌 / -우) |
| `brake` | `uint8` | 0~150 | 제동 강도 |

### 2. Feedback.msg
차량 상태 피드백 메시지

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `header` | `std_msgs/Header` | - | 메시지 헤더 |
| `manual_mode` | `bool` | Manual/Auto | 제어 모드 (true=수동, false=자동) |
| `emergency_stop` | `bool` | On/Off | E-Stop 상태 (true=활성) |
| `gear` | `uint8` | 0/1/2 | 기어 (0=DRIVE, 1=NEUTRAL, 2=REVERSE) |
| `speed` | `float64` | m/s | 현재 선속도 |
| `steering` | `float64` | rad | 현재 조향각 (+좌 / -우) |
| `brake` | `uint8` | 0~150 | 현재 제동 강도 |
| `encoder_count` | `int32` | - | 휠 엔코더 카운터 |
| `heartbeat` | `uint8` | 0~255 | 통신 상태 확인용 (매 사이클 +1) |

## 서비스 정의

### 1. ModeCommand.srv
모드/E-Stop/기어 설정 서비스

**Request**

| 필드 | 타입 | 설명 |
|---|---|---|
| `manual_mode` | `bool` | 수동 모드 활성화 여부 |
| `emergency_stop` | `bool` | E-Stop 활성화 여부 |
| `gear` | `uint8` | 기어 설정 (0=DRIVE, 1=NEUTRAL, 2=REVERSE) |

**Response**

| 필드 | 타입 | 설명 |
|---|---|---|
| `success` | `bool` | 명령 수행 성공 여부 |
