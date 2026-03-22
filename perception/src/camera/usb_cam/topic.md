# usb_cam 패키지 토픽 정리

## 구독 토픽
없음 (카메라 드라이버 -- 소스 노드)

## 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `image_raw` | `sensor_msgs/msg/Image` | depth=100 | 카메라 원본 이미지 (640x480, MJPEG->RGB) |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | depth=100 | 카메라 캘리브레이션 정보 |
| `image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | depth=100 | MJPEG 압축 이미지 (pixel_format=mjpeg일 때만) |

## 서비스
| 서비스 이름 | 서비스 타입 | 설명 |
|---|---|---|
| `set_capture` | `std_srvs/srv/SetBool` | 프레임 캡처 시작/정지 |
