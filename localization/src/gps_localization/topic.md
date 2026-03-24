# gps_localization 패키지 토픽 정리

## utm_localizer_node

### 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | depth=10 | GPS 원시 데이터 (위도, 경도, 공분산) |

### 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 프레임 | 설명 |
|---|---|---|---|---|
| `/local_path` | `visualization_msgs/msg/Marker` (POINTS) | depth=10 | base_link | 웨이포인트 경로 (차량 로컬 좌표) |

---

## waypoint_recorder_node

### 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | depth=10 | GPS 원시 데이터 (웨이포인트 기록용) |

### 발행 토픽

없음 (CSV 파일 출력만)
