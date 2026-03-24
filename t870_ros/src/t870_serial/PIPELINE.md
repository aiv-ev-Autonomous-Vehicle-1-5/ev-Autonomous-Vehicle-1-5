# t870_serial 패키지 파이프라인

## 패키지 정보
- **Package**: t870_serial
- **Description**: Henes T870 시리얼 통신 브릿지 -- ROS2 메시지 <-> T870 PCU 시리얼 패킷 변환
- **Node**: serial_bridge (ComposableNode)

---

## 통신 사양

| 항목 | 값 |
|------|-----|
| 시리얼 프로토콜 | 115200 baud, 8N1 |
| TX/RX 패킷 크기 | 13바이트 |
| 패킷 헤더 | STX (0x53, 0x54, 0x58) |
| 패킷 종결 | ETX (0x0D, 0x0A) |
| 타이머 주기 | 20ms (50Hz) |
| 명령 타임아웃 | 0.5초 (미수신 시 속도/조향 리셋) |

---

## 변환 계수

| 계수 | 값 | 설명 |
|------|-----|------|
| MPS2BYTE | 555.56 | m/s -> 바이트 변환 |
| BYTE2MPS | 0.000196 | 바이트 -> m/s 변환 |
| RAD2BYTE | -5729.58 | rad -> 바이트 변환 |
| BYTE2RAD | -0.000175 | 바이트 -> rad 변환 |

---

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `port_path` | string | `/dev/ttyUSB0` | 시리얼 포트 경로 |
| `baud_rate` | int | 115200 | 시리얼 통신 속도 |
| `max_speed_mps` | double | 1.60 | 최대 속도 (m/s) |
| `max_steering_deg` | double | 18.00 | 최대 조향각 (deg) |
| `steering_offset_deg` | double | 0.00 | 조향 오프셋 (deg) |

---

## 데이터 흐름

```
[제어 경로 (ROS -> PCU)]
  /t870/control_command (ControlCommand)
    -> 타임아웃 검사 (0.5초)
    -> 속도/조향 클램핑 (max_speed_mps, max_steering_deg)
    -> MPS2BYTE, RAD2BYTE 변환
    -> TX 패킷 조립 (STX + 데이터 + ETX)
    -> 시리얼 전송

[피드백 경로 (PCU -> ROS)]
  시리얼 수신
    -> RX 패킷 파싱 (STX/ETX 검증)
    -> BYTE2MPS, BYTE2RAD 변환
    -> /t870/feedback (Feedback) 발행

[모드 설정 (서비스)]
  /t870/mode_command (ModeCommand.srv)
    -> TX 패킷에 모드/E-Stop/기어 반영
```
