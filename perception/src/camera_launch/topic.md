# camera_launch 패키지 토픽 정리 (전체 파이프라인)

## 파이프라인 토픽 흐름 (camera_all.launch.py)
| 단계 | 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|---|
| 1. USB Camera | `/camera1/image_raw` | `sensor_msgs/msg/Image` | depth=100 | 원본 카메라 이미지 (640x480) |
| 1. USB Camera | `/camera1/camera_info` | `sensor_msgs/msg/CameraInfo` | depth=100 | 카메라 캘리브레이션 |
| 2. BEV 변환 | `/bev_image` | `sensor_msgs/msg/Image` | depth=10 | BEV 변환 이미지 (400x400) |
| 3. 차선 인식 | `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | 최종 차선 경계 -> planning |

## TF 발행
| parent_frame | child_frame | 변환 |
|---|---|---|
| `base_link` | `camera1` | x=0.224m, y=0m, z=1.264m |
