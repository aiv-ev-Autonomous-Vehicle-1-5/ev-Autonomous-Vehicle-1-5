# dbscan_clustering 패키지 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/patchworkpp/nonground` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 비지면 포인트클라우드 (Patchwork++ 출력) |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/pointcloud/clustered` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 클러스터링된 포인트클라우드 (cluster_id + rgb 필드 포함) |
