# make_bbox 패키지 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/pointcloud/clustered` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 클러스터링된 포인트클라우드 (cluster_id 필드 포함) |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/raw_bboxes` | `ev_msgs/msg/BBoxArray` | SensorDataQoS | 원시 바운딩 박스 배열 |
| `/perception/bboxes_marker` | `visualization_msgs/msg/MarkerArray` | SensorDataQoS | 바운딩 박스 시각화 마커 |
