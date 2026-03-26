# lidar_launch 패키지 토픽 정리 (전체 파이프라인)

## 파이프라인 토픽 흐름

| 단계 | 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|---|
| 1. Driver | `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | Velodyne VLP-16 원시 데이터 |
| 2. Ground Seg | `/patchworkpp/nonground` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 비지면 포인트 |
| 2. Ground Seg | `/patchworkpp/ground` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 지면 포인트 |
| 3. Clustering | `/pointcloud/clustered` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | 클러스터링 결과 (cluster_id + rgb) |
| 4. BBox | `/perception/bboxes` | `ev_msgs/msg/BBoxArray` | SensorDataQoS | 바운딩 박스 → planning 직접 구독 |
| 4. BBox | `/perception/bboxes_marker` | `visualization_msgs/msg/MarkerArray` | SensorDataQoS | BBox 시각화 |
