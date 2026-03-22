# lidar_voxel_grid — PCL VoxelGrid 다운샘플링

## 개요

PCL VoxelGrid 기반 포인트클라우드 다운샘플링. 근거리(near_range 이내)만 복셀 그리드 필터링하고, 원거리는 원본을 그대로 패스스루한다.

- Node: `VoxelGridComponent` (ComposableNode)

## 듀얼존 처리

```
입력 PointCloud2
    │
    ├─ XY 거리 ≤ near_range → VoxelGrid 필터 적용 (다운샘플링)
    ├─ XY 거리 > near_range  → 원본 유지 (패스스루)
    │
    ▼
출력 PointCloud2 (근거리 다운샘플 + 원거리 원본)
```

## 파라미터

| 파라미터 | 기본값 | 단위 | 설명 |
|---------|--------|------|------|
| `leaf_size_x` | 0.05 | m | 복셀 X 크기 |
| `leaf_size_y` | 0.05 | m | 복셀 Y 크기 |
| `leaf_size_z` | 0.05 | m | 복셀 Z 크기 |
| `near_range` | 1.5 | m | 근거리 판정 거리 (XY 유클리드) |

## 소스 파일

| 파일 | 역할 |
|------|------|
| `include/lidar_voxel_grid/voxel_grid_component.hpp` | 컴포넌트 클래스 선언 |
| `src/voxel_grid_component.cpp` | VoxelGrid 필터링 로직 구현 |
| `config/voxel_grid.yaml` | 로컬 파라미터 (참고용) |
