# lidar_launch — LiDAR 인식 파이프라인 런치 메타패키지

## 개요

LiDAR 인식 파이프라인 런치 및 설정 메타패키지. 노드 없이 런치 파일과 중앙집중 설정만 제공한다.

## 전체 파이프라인 데이터 흐름

```
Velodyne VLP-16 (UDP)
    │
    ├─ velodyne_driver (패킷 수신)
    ├─ velodyne_transform (패킷 → PointCloud2)
    │
    ▼
/velodyne_points (PointCloud2)
    │
    ├─ Patchwork++ (지면 분리)
    │   ├─ /patchworkpp/ground     (지면)
    │   └─ /patchworkpp/nonground  (비지면)
    │
    ▼
/patchworkpp/nonground
    │
    ├─ VoxelGrid (optional, 근거리 다운샘플링)
    │
    ├─ DBSCAN Clustering (클러스터링)
    │   └─ /pointcloud/clustered (PointCloud2, cluster_id + rgb)
    │
    ▼
/pointcloud/clustered
    │
    ├─ make_bbox (클러스터 → BBox 변환 + 크기 필터 + 분할)
    │   ├─ /perception/bboxes        (ev_msgs/BBoxArray)  ← planning 직접 구독
    │   └─ /perception/bboxes_marker (MarkerArray)        ← 시각화
    │
    ▼
/perception/bboxes → planning (chaining_costmap_ver)

    [DEPRECATED: bbox_tracker는 파이프라인에서 제거됨]
    [make_bbox가 /perception/bboxes로 직접 발행 → planning이 직접 구독]
```

## 런치 파일

| 파일 | 용도 |
|------|------|
| `my_velodyne_VLP16-composed-launch.py` | 실차 파이프라인 |
| `sim_velodyne_VLP16-composed-launch.py` | 시뮬레이션 파이프라인 |

## 설정 파일

모든 perception config는 `lidar_launch/config/`에 집중 관리:

```
lidar_launch/config/
├── patchworkpp/
│   └── patchworkpp_params.yaml
├── dbscan_clustering/
│   └── dbscan_params.yaml
├── lidar_voxel_grid/
│   └── voxel_grid_params.yaml
└── make_bbox/
    └── make_bbox_params.yaml
```

## 노드별 토픽

### 1. Velodyne Driver + Transform

| 방향 | 토픽 | 타입 |
|------|------|------|
| Pub | `/velodyne_points` | `sensor_msgs/PointCloud2` |

### 2. Patchwork++ (지면 분리)

| 방향 | 토픽 | 타입 |
|------|------|------|
| Sub | `/velodyne_points` | `sensor_msgs/PointCloud2` |
| Pub | `/patchworkpp/ground` | `sensor_msgs/PointCloud2` |
| Pub | `/patchworkpp/nonground` | `sensor_msgs/PointCloud2` |

### 3. DBSCAN Clustering

| 방향 | 토픽 | 타입 |
|------|------|------|
| Sub | `/patchworkpp/nonground` | `sensor_msgs/PointCloud2` |
| Pub | `/pointcloud/clustered` | `sensor_msgs/PointCloud2` |

### 4. make_bbox (클러스터 → BBox)

| 방향 | 토픽 | 타입 |
|------|------|------|
| Sub | `/pointcloud/clustered` | `sensor_msgs/PointCloud2` |
| Pub | `/perception/bboxes` | `ev_msgs/BBoxArray` |
| Pub | `/perception/bboxes_marker` | `visualization_msgs/MarkerArray` |

### ~~5. bbox_tracker (트래킹)~~ [DEPRECATED]

> bbox_tracker는 파이프라인에서 제거되었습니다. make_bbox가 `/perception/bboxes`로 직접 발행합니다.

## 메시지 타입

### ev_msgs/BBox
```
geometry_msgs/Point position    # 중심 (x, y, z) [m]
float32 size_x                  # X 크기 [m]
float32 size_y                  # Y 크기 [m]
float32 size_z                  # Z 크기 (높이) [m]
int32 label                     # 클러스터 ID
```

### ev_msgs/BBoxArray
```
std_msgs/Header header
BBox[] bboxes
```

## tmuxp 실행 순서

```yaml
# ev.yaml / ev_real.yaml
- Gazebo / T870 serial          # 차량
- lidar_launch make_bbox        # perception 파이프라인
- chaining_costmap_ver          # planning
- pure_pursuit                  # control
```
