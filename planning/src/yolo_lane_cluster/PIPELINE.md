# yolo_lane_cluster — 차선 전처리 파이프라인

## 패키지 개요

- **Package**: yolo_lane_cluster
- **Description**: 카메라 차선 전처리 — backbone chaining 기반 좌/우 차선 생성 + 가상 차선 생성
- ROS 2 Humble, C++17, Component architecture
- **Node**: YoloLaneClusterNode (ComposableNode, 이벤트 기반)

---

## 데이터 흐름

```
yolo_instance_seg_node
    │  /perception/raw_lane_boundaries (LaneBoundaryArray, Best Effort)
    ▼
yolo_lane_cluster_node
    │  /perception/lane_boundaries (LaneBoundaryArray, Best Effort)
    │  /yolo_lane_cluster/debug/lane_points (MarkerArray, lazy)
    ▼
chaining_costmap_ver (변경 없음)
```

---

## 처리 로직

### Step 1: 포인트 풀 생성
- 카메라에서 받은 모든 클러스터의 포인트를 하나의 LanePoint 벡터로 flat화
- 각 포인트에 원래 클러스터(boundary) 인덱스를 label로 저장

### Step 2: Seed 찾기
- 초기 seed 위치: LEFT (0, +0.8m), RIGHT (0, -0.8m)
- 포인트 풀에서 각 seed 좌표에 유클리디안 거리가 가장 가까운 포인트를 seed로 선택
- 클러스터 x_min 포인트의 y 부호로 좌/우 판별 (y>0 → 왼쪽, y<0 → 오른쪽)
- **Seed 타임아웃**: chaining이 `seed.timeout_sec`(기본 3초) 동안 미성공 시 해당 seed를 초기 위치로 리셋
- seed 업데이트 시 y가 반대쪽으로 drift하면 초기값으로 리셋

### Step 3: Backbone Chaining (Greedy Forward)
- LEFT seed에서 전방(+x) 방향으로 greedy chaining → LEFT backbone
- RIGHT seed에서 전방(+x) 방향으로 greedy chaining → RIGHT backbone
- chaining_costmap_ver의 DirectionChainer를 간소화하여 이식

#### 비용함수
- 기본: `w = α·C_d + β·C_a + γ·C_lat`
  - C_d: 거리 비용 (d / d_max)
  - C_a: 방향 오차 비용 (angle / θ_max)
  - C_lat: 횡오차 비용 (lateral_offset / lateral_gate)
- 확장: `w' = w + λ·C_side`
  - C_side: 중심선 쪽 이동 페널티 (LEFT: y 감소 시, RIGHT: y 증가 시)

#### Gate 시스템
- G1 (거리 게이트): `d(i,j) ≤ d_max`
- G2 (전방 cone 게이트): `angle(v, u_ij) ≤ forward_cone_deg/2`
- G3 (횡오차 게이트): 편측 — LEFT는 안쪽(오른쪽) 초과 제한, RIGHT는 안쪽(왼쪽) 초과 제한

#### 클러스터 락
- 특정 클러스터 포인트가 선택되면 해당 클러스터만 후보로 제한
- 클러스터 내 후보 소진 시 락 해제 → 다음 클러스터로 chaining 확장

### Step 4: Overlap 해소 (Backtracking)
- LEFT/RIGHT backbone에서 공통 노드(중복) 탐지
- 3-node 윈도우 곡률+거리 비용 비교
- 비용 높은 쪽 truncate + 중복 노드 제외 후 재chaining
- 비용 동일 시 양쪽 truncate
- max_backtrack_count까지 반복

### Step 5: 출력 구성 (기존 로직 유지)
- **양쪽 다 chaining 성공**: 총 경로 길이가 긴 쪽을 채택, track_width 안쪽 오프셋으로 반대편 가상 차선 생성
- **한쪽만 chaining 성공**: 반대편 가상 차선 생성
- 가상 차선에 filter_virtual_lane_outliers() 적용 (인접 세그먼트 간 각도 변화 > 30° 포인트 제거)
- 모든 boundary에 lane_side 라벨 설정 (SIDE_LEFT=1 / SIDE_RIGHT=2)
- 가상 차선 lane_id = -1

### Step 6: 발행 + 디버그
- 실제 + 가상 boundary를 LaneBoundaryArray로 발행
- lazy 디버그 마커: 고정 seed 위치 (구), 실제/가상 차선 points

---

## 디렉토리 구조

```
yolo_lane_cluster/
├── include/yolo_lane_cluster/
│   └── yolo_lane_cluster_node.hpp    # 노드 선언 + LanePoint/ChainerParams + chaining 메서드
├── src/
│   ├── yolo_lane_cluster_node.cpp    # 노드 생성자 + 콜백 오케스트레이션
│   ├── lane_chainer.cpp              # backbone chaining (find_seed, extract, cost, backtrack)
│   ├── virtual_lane_gen.cpp          # 가상 차선 생성 (track_width 오프셋) + outlier 필터링
│   └── debug_publisher.cpp           # 디버그 마커 발행 (lazy)
├── config/
│   └── yolo_lane_cluster.yaml        # 파라미터
├── launch/
│   └── yolo_lane_cluster.launch.py   # 런치 파일
├── CMakeLists.txt
└── package.xml
```

---

## 주요 파라미터

설정 파일: `config/yolo_lane_cluster.yaml`

### seed

| 파라미터 | 기본값 | 단위 | 설명 |
|----------|--------|------|------|
| `left_y` | 0.8 | m | 왼쪽 시드 y (초기 위치) |
| `right_y` | -0.8 | m | 오른쪽 시드 y (초기 위치) |
| `timeout_sec` | 3.0 | s | chaining 미성공 시 seed 초기 위치 리셋 타임아웃 |

### chainer

| 파라미터 | 기본값 | 단위 | 설명 |
|----------|--------|------|------|
| `d_max` | 2.5 | m | 탐색 최대 거리 |
| `forward_cone_deg` | 120.0 | deg | 전방 cone 전체 각도 |
| `lateral_gate` | 1.3 | m | 횡방향 오차 한계 |
| `alpha` | 1.2 | - | 거리 비용 가중치 |
| `beta` | 1.2 | - | 방향 오차 비용 가중치 |
| `gamma` | 0.7 | - | 횡오차 비용 가중치 |
| `lambda_side` | 0.5 | - | side preference 가중치 |
| `max_backtrack_count` | 5 | - | 중복 해소 최대 반복 |
| `backtrack_w_curv` | 1.0 | - | backtracking 곡률 가중치 |
| `backtrack_w_dist` | 1.0 | - | backtracking 거리 가중치 |
| `max_chain_len` | 300 | - | backbone 최대 길이 |

### virtual_lane

| 파라미터 | 기본값 | 단위 | 설명 |
|----------|--------|------|------|
| `track_width` | 1.6 | m | 트랙 폭 (가상 차선 오프셋 거리) |

---

## 빌드 명령

```bash
cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select yolo_lane_cluster
```
