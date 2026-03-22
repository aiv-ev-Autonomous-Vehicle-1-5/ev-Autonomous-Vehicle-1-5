# camera_launch 파이프라인

## 패키지 정보
- **Package**: camera_launch
- **Description**: 카메라 인식 파이프라인 런치 메타패키지
- **노드 없음** -- 런치 파일만 제공

## 런치 파일

### 1. camera.launch.py
USB 카메라 + TF (원본 usb_cam 런치)

### 2. camera_all.launch.py (주 사용)
전체 카메라 파이프라인 통합 런치 파일.

포함 노드:
- `usb_cam_node_exe` (camera1) + `static_transform_publisher` (base_link -> camera1)
- `bev_lut_node` (BEV 변환)
- `yolo_db_seg_node` (차선 인식)

## 전체 데이터 흐름

```
USB Camera
    |
    v
/camera1/image_raw
    |
    v
bev_lut_node
    |
    v
/bev_image
    |
    v
yolo_db_seg_node
    |
    v
/perception/lane_boundaries
```

**파이프라인**: USB Camera -> `/camera1/image_raw` -> `bev_lut_node` -> `/bev_image` -> `yolo_db_seg_node` -> `/perception/lane_boundaries`

## TF

| parent_frame | child_frame | 변환 |
|---|---|---|
| `base_link` | `camera1` | x=0.224m, y=0m, z=1.264m |
