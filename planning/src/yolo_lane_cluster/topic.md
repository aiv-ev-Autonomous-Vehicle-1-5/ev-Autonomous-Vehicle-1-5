# yolo_lane_cluster 토픽 정리

## 구독 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/raw_lane_boundaries` | `ev_msgs/LaneBoundaryArray` | Best Effort, depth=1 | 카메라 원본 차선 (yolo_instance_seg_node) |

## 발행 토픽

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/LaneBoundaryArray` | Best Effort, depth=1 | 가공된 차선 (실제 + 가상) → planning |
| `/yolo_lane_cluster/debug/lane_points` | `visualization_msgs/MarkerArray` | Best Effort, depth=1, lazy | 디버그 시각화 (실제/가상 차선 + 시드 위치 + 탐색 영역) |
