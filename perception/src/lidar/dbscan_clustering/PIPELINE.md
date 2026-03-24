# dbscan_clustering — GPU 가속 DBSCAN 클러스터링

## 개요

GPU 가속 DBSCAN 클러스터링 노드. LiDAR 비지면 포인트를 클러스터링하여 cluster_id와 HSV 기반 RGB 컬러를 할당한다.

- CUDA GPU 지원 (없으면 nanoflann KD-Tree CPU 폴백)
- Node: `DBSCANNode` (ComposableNode)

## 빌드

- CUDA 자동 감지, `dbscan_component` 라이브러리로 빌드

## 알고리즘: DBSCAN

```
1. eps-이웃 탐색 — 각 포인트의 eps 반경 내 이웃 수 계산
2. 코어 포인트 판별 — 이웃 수 ≥ min_points인 포인트를 코어로 분류
3. 클러스터 확장 — 코어 포인트로부터 연결된 포인트들을 동일 클러스터로 병합
4. 노이즈 할당 — 어떤 클러스터에도 속하지 않는 포인트에 cluster_id = -1 부여
5. 컬러 할당 — cluster_id별 HSV 기반 RGB 컬러 생성
```

## 출력 포인트 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | FLOAT32 | X 좌표 |
| `y` | FLOAT32 | Y 좌표 |
| `z` | FLOAT32 | Z 좌표 |
| `cluster_id` | INT32 | 클러스터 ID (-1 = 노이즈) |
| `rgb` | FLOAT32 | HSV 기반 RGB 컬러 |

## 파라미터

### 핵심 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `eps` | 0.15 | DBSCAN 이웃 탐색 반경 (m) |
| `min_points` | 10 | 코어 포인트 최소 이웃 수 |
| `max_neighbors` | 1024 | 최대 이웃 수 |
| `max_points` | 80000 | 최대 처리 포인트 수 |

### ROI (관심 영역)

| 파라미터 | 범위 | 단위 |
|---------|------|------|
| `x` | [-4.0, 14.0] | m |
| `y` | [-7.0, 7.0] | m |
| `z` | [-2.0, 0.0] | m |

### 방위각 지면 억제

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `enable_azimuth_ground_suppression` | true | 방위각 지면 억제 활성화 |
| `azimuth_ground_z_threshold` | 0.01 | 지면 판정 Z 임계값 (m) |

### 클러스터링 옵션

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `cluster_xy_only` | false | XY 평면만으로 클러스터링 여부 |
| `z_weight` | 0.3 | Z 축 거리 가중치 |
| `z_distance_scale` | 0.05 | Z 거리 스케일 |

## 소스 파일

| 파일 | 역할 |
|------|------|
| `include/dbscan_clustering/dbscan_node.hpp` | 노드 클래스 선언 |
| `src/dbscan_node.cpp` | DBSCAN 클러스터링 로직 구현 |
| `config/dbscan.yaml` | 로컬 파라미터 (참고용) |
