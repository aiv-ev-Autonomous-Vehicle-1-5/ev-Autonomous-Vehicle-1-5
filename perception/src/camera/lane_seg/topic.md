# lane_seg 패키지 토픽 정리

## bev_lut_node

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/camera1/image_raw` | `sensor_msgs/msg/Image` | depth=10 | USB 카메라 원본 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 변환 이미지 (400x400) |

## yolo_seg_node

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/yolo_seg_image` | `sensor_msgs/msg/Image` | depth=10 | 세그멘테이션 시각화 이미지 |
| `/lane_coordinates` | `lane_seg_msgs/msg/LaneCoords` | depth=10 | 단순 차선 좌표 (line_x[], line_y[]) |

## yolo_db_seg_node (주 사용 노드)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | 구조화된 차선 경계 (base_link 프레임) |

## lane_coord_viewer (디버그)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/lane_coordinates` | `lane_seg_msgs/msg/LaneCoords` | depth=10 | 차선 좌표 (콘솔 출력용) |
