# make_bbox — 클러스터 → BBox 변환

## 개요

클러스터링된 PointCloud2를 BBoxArray로 변환한다. K-means 클러스터 분할 및 크기 필터링을 포함한다.

- Node: `MakeBBoxNode` (ComposableNode)

## 처리 파이프라인

```
1. 클러스터링된 포인트클라우드 수신 (cluster_id 필드)
       │
       ▼
2. cluster_id별 포인트 그룹핑
       │
       ▼
3. K-means 기반 클러스터 분할
   (enable_cluster_split, split_cone_diameter_m = 0.5)
       │
       ▼
4. 바운딩 박스 생성 (center, size_x/y/z)
       │
       ▼
5. 크기 필터링
   (max_size_x = 1.75, max_size_y = 1.75, max_size_z = 1.5)
       │
       ▼
6. BBoxArray + MarkerArray 발행
```

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `max_size_x` | 1.75 | BBox X 최대 크기 (m) |
| `max_size_y` | 1.75 | BBox Y 최대 크기 (m) |
| `max_size_z` | 1.5 | BBox Z 최대 크기 (m) |
| `enable_cluster_split` | true | K-means 클러스터 분할 활성화 |
| `split_cone_diameter_m` | 0.5 | 분할 기준 콘 직경 (m) |
| `split_kmeans_max_iter` | 15 | K-means 최대 반복 횟수 |
| `split_min_points` | 0 | 분할 최소 포인트 수 |

## 소스 파일

| 파일 | 역할 |
|------|------|
| `include/make_bbox/make_bbox_node.hpp` | 노드 클래스 선언 |
| `src/make_bbox_node.cpp` | BBox 변환 및 필터링 로직 구현 |
