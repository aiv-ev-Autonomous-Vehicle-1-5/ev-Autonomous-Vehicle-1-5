# Camera 패키지 토픽 정리

## usb_cam (C++ 노드)

카메라 namespace: `camera1`

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/camera1/image_raw` | `sensor_msgs/msg/Image` | depth=100 | USB 카메라 원본 이미지 (Logitech C930, 640x480) |
| `/camera1/camera_info` | `sensor_msgs/msg/CameraInfo` | depth=100 | 카메라 캘리브레이션 정보 |

---

## bev_lut_node (Python)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/camera1/image_raw` | `sensor_msgs/msg/Image` | depth=10 | USB 카메라 원본 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 변환 이미지 (400x400), LUT 기반 호모그래피 |

---

## yolo_instance_seg_node (Python, 주 사용 노드)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/raw_lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | 원본 차선 경계 (인스턴스별 lane_id 포함) -> yolo_lane_cluster 노드로 전달 |
| `/yolo_instance_seg_image` | `sensor_msgs/msg/Image` | depth=10 | 인스턴스 세그멘테이션 시각화 이미지 (디버그용) |

---

## yolo_db_seg_node (Python, 미사용)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | DBSCAN 클러스터링 차선 경계 |
| `/yolo_db_seg_image` | `sensor_msgs/msg/Image` | depth=10 | DBSCAN 세그멘테이션 시각화 이미지 |

---

## yolo_seg_node (Python, 미사용)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 이미지 |

### 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/yolo_seg_image` | `sensor_msgs/msg/Image` | depth=10 | 세그멘테이션 시각화 이미지 |
| `/lane_coordinates` | `lane_seg_msgs/msg/LaneCoords` | depth=10 | 단순 차선 좌표 (line_x[], line_y[]) |

---

## lane_coord_viewer (Python, 디버그 유틸)

### 구독 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/lane_coordinates` | `lane_seg_msgs/msg/LaneCoords` | depth=10 | 차선 좌표 (콘솔 출력용) |

---

## 데이터 흐름

```
usb_cam (/camera1/image_raw)
    |
    v
bev_lut_node (/bev_image)
    |
    v
yolo_instance_seg_node (/perception/raw_lane_boundaries)
    |
    v
yolo_lane_cluster_node (/perception/lane_boundaries)
    |
    v
chaining_costmap_ver (planning)
```
