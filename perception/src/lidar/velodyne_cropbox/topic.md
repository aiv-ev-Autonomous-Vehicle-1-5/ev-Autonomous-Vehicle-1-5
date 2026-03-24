# velodyne_cropbox 패키지 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `input` (리맵 가능) | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 입력 포인트클라우드 |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `output` (리맵 가능) | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 필터링된 포인트클라우드 |
