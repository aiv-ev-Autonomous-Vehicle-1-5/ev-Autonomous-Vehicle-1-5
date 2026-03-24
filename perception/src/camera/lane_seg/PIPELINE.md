# lane_seg 파이프라인

## 패키지 정보
- **Package**: lane_seg
- **Description**: YOLO + BEV(Bird's Eye View) 기반 차선 인식 패키지 (Python, ament_python)

## 노드

### 1. bev_lut_node
카메라 이미지를 BEV(Bird's Eye View)로 변환하는 노드. 호모그래피 LUT(Look-Up Table)를 사용한다.

- **카메라**: Logitech C930, 640x480
- **BEV 출력**: 400x400
- **변환 행렬**: `config/bev_matrix.npy`에서 로드

### 2. yolo_seg_node
BEV 이미지에서 YOLO 세그멘테이션을 수행하여 단순 차선 좌표를 출력하는 노드.

- **모델**: `second_best.pt`, conf=0.7, imgsz=320
- **좌표 변환**: BEV 기준점(226, 419), 1px=0.01m

### 3. yolo_db_seg_node (주 사용 노드)
BEV 이미지에서 YOLO + DBSCAN 클러스터링을 수행하여 구조화된 차선 경계를 출력하는 노드.

- **클러스터링**: 1D gap-sort 클러스터링 (eps=30, min_samples=50)
- **출력**: `LaneBoundaryArray` (각 boundary에 ordered `Point[]` + confidence)
- **CUDA 지원** (CPU 폴백)

### 4. lane_coord_viewer
디버깅 유틸리티 노드 (콘솔 출력).

## 데이터 흐름

```
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

**전체 파이프라인**: `/camera1/image_raw` -> `bev_lut_node` -> `/bev_image` -> `yolo_db_seg_node` -> `/perception/lane_boundaries`
