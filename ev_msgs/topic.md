# ev_msgs 패키지 토픽 정리

이 패키지는 메시지 정의만 포함하며, ROS 토픽을 직접 구독/발행하지 않습니다.

## 정의된 메시지 타입

| 메시지 타입 | 필드 | 설명 |
|---|---|---|
| `BBox` | `geometry_msgs/Point position`, `float32 size_x/y/z`, `int32 label` | 단일 바운딩 박스 |
| `BBoxArray` | `std_msgs/Header header`, `BBox[] bboxes` | 바운딩 박스 배열 |
| `LaneBoundary` | `std_msgs/Header header`, `geometry_msgs/Point[] points`, `float32 confidence` | 단일 차선 경계 |
| `LaneBoundaryArray` | `std_msgs/Header header`, `LaneBoundary[] boundaries` | 차선 경계 배열 |

## 사용하는 토픽

| 토픽 이름 | 메시지 타입 | 발행 노드 | 구독 노드 |
|---|---|---|---|
| `/perception/bboxes` | `BBoxArray` | bbox_tracker | chaining_costmap_ver, pure_pursuit |
| `/perception/raw_bboxes` | `BBoxArray` | make_bbox | bbox_tracker |
| `/perception/lane_boundaries` | `LaneBoundaryArray` | yolo_db_seg_node | chaining_costmap_ver |
