# yolo_lane_cluster — 차선 전처리 파이프라인

## 패키지 개요

- **Package**: yolo_lane_cluster
- **Description**: 카메라 차선 전처리 — 시드 기반 좌/우 판별 + 가상 차선 생성
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
chaining_costmap_ver (기존 로직 변경 없음)
```

---

## 처리 로직

### Step 1: 시드 매칭
- 시드 2개: 왼쪽 (초기 y=+0.75m), 오른쪽 (초기 y=-0.75m)
- 직사각형 탐색 영역 (width × height, 파라미터) 내에서 가장 가까운 클러스터 매칭
- 왼쪽 시드에 가장 가까운 클러스터 = 왼쪽 차선
- 오른쪽 시드에 가장 가까운 클러스터 = 오른쪽 차선 (왼쪽과 동일 클러스터 제외)

### Step 2: 시드 추적
- 매칭 성공 시: 시드 중심 x = 매칭된 클러스터의 x_min (y는 초기값 유지)
- 매칭 실패 시: 시드 중심 마지막 위치 유지
- 차선 소실 후 재등장 시 마지막 시드 위치 근처에서 매칭

### Step 3: 가상 차선 생성
- 한쪽만 매칭 시 반대편 가상 차선 생성
- 각 point의 tangent(방향벡터) 수직으로 track_width(1.5m) 안쪽 오프셋
- 가상 차선 lane_id = -1

### Step 4: 출력
- 실제 + 가상 boundary를 LaneBoundaryArray로 발행
- planning이 기존 로직 그대로 처리 (backbone chaining → centerline → goal)

---

## 디렉토리 구조

```
yolo_lane_cluster/
├── include/yolo_lane_cluster/
│   └── yolo_lane_cluster_node.hpp    # 노드 선언 + 타입 + 파라미터
├── src/
│   ├── yolo_lane_cluster_node.cpp    # 노드 생성자 + 콜백 오케스트레이션
│   ├── seed_tracker.cpp              # 시드 매칭/업데이트 로직
│   ├── virtual_lane_gen.cpp          # 가상 차선 생성 (1.5m 오프셋)
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
| `init_x` | 0.0 | m | 시드 초기 x 위치 |
| `left_y` | 0.75 | m | 왼쪽 시드 y 오프셋 |
| `right_y` | -0.75 | m | 오른쪽 시드 y 오프셋 |
| `search_rect_width` | 2.0 | m | 탐색 직사각형 y 방향 폭 |
| `search_rect_height` | 8.0 | m | 탐색 직사각형 x 방향 높이 |

### virtual_lane

| 파라미터 | 기본값 | 단위 | 설명 |
|----------|--------|------|------|
| `track_width` | 1.5 | m | 트랙 폭 (가상 차선 오프셋 거리) |

---

## 빌드 명령

```bash
cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select yolo_lane_cluster
```
