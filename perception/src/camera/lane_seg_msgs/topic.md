# lane_seg_msgs 패키지 토픽 정리

이 패키지는 메시지 정의만 포함하며, ROS 토픽을 직접 구독/발행하지 않습니다.

## 정의된 메시지 타입

| 메시지 타입 | 필드 | 설명 |
|---|---|---|
| `LaneCoords` | `float32[] line_x`, `float32[] line_y` | 차선 포인트 좌표 배열 (m 단위) |

## 사용하는 토픽
| 토픽 이름 | 메시지 타입 | 발행 노드 | 구독 노드 |
|---|---|---|---|
| `/lane_coordinates` | `lane_seg_msgs/msg/LaneCoords` | `yolo_seg_node` | `lane_coord_viewer` |
