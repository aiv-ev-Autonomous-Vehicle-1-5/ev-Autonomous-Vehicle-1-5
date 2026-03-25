# bbox_tracker — 자전거 모델 기반 BBox 트래킹

## 개요

자전거 모델 기반 자아운동 예측을 활용한 BBox 트래킹 노드. LiDAR 최소 인식거리 사각지대에 진입한 라바콘을 ego-motion(조향/속도) 기반으로 예측 유지하여, 인식 끊김 없는 연속적인 bbox 데이터를 planning에 제공한다.

- Node: `BBoxTrackerNode` (ComposableNode)

## 파이프라인 위치

```
perception (make_bbox)
    │
    ├─ /perception/raw_bboxes (ev_msgs/BBoxArray)
    │
    ▼
bbox_tracker
    │   ← /t870/control_command (t870_msgs/ControlCommand)
    │
    ├─ /perception/bboxes (ev_msgs/BBoxArray)  ← planning이 구독
    │
    ├─ /tracker/debug/tracks    (Marker POINTS, lazy)     ← 전체 트랙 시각화
    └─ /tracker/debug/predicted (Marker CUBE_LIST, lazy)  ← 예측 유지 트랙만
```

planning 코드 변경 없이, 토픽명만 중간에 remap하여 파이프라인에 삽입.

## 알고리즘 — 트래킹 사이클 (bbox 수신 시마다 실행)

```
┌─────────────────────────────────────────────────────┐
│  1. Predict — Bicycle Model로 ego-motion 계산       │
│     dθ = (v × tan(δ) / L) × dt                     │
│     dx = v × cos(dθ/2) × dt                        │
│     dy = v × sin(dθ/2) × dt                        │
│     → 기존 트랙을 현재 base_link 좌표로 역변환      │
│     → 정지(v < 1e-4) 시 skip                        │
│                                                     │
│  2. Match — Greedy Nearest Neighbor                 │
│     동적 threshold = max(min_match_dist, |v| × dt)  │
│     → 새 검출과 기존 트랙을 최근접 거리로 매칭       │
│                                                     │
│  3. Update — 트랙 상태 갱신                          │
│     ├─ 매칭 성공: 트랙을 검출값으로 갱신, miss=0     │
│     ├─ 미매칭 트랙: predict 위치 유지, miss++        │
│     ├─ miss > max_miss_count: 트랙 삭제              │
│     └─ 미매칭 검출: 새 트랙 생성                     │
│                                                     │
│  4. Publish — 모든 활성 트랙을 BBoxArray로 발행      │
│     └─ 디버그 마커 발행 (lazy)                       │
└─────────────────────────────────────────────────────┘
```

## Predict — Bicycle Model 상세

```
자차 이동량:
  dθ = (v × tan(δ) / wheelbase) × dt    ← 회전량
  dx = v × cos(dθ/2) × dt               ← 전방 이동
  dy = v × sin(dθ/2) × dt               ← 횡방향 이동

기존 트랙 역변환 (자차가 앞으로 → 트랙은 상대적으로 뒤로):
  rx = track.x - dx
  ry = track.y - dy
  track.x =  cos(dθ) × rx + sin(dθ) × ry
  track.y = -sin(dθ) × rx + cos(dθ) × ry
```

## 동적 매칭 threshold

```
threshold = max(min_match_dist, |speed| × dt)

속도 0.0m/s → 0.1m (최소값)
속도 0.5m/s, dt=0.1s → 0.1m (최소값)
속도 1.0m/s, dt=0.1s → 0.1m
속도 2.0m/s, dt=0.1s → 0.2m
```

## 고정 Miss Count 기반 트랙 삭제

매칭 실패(miss) 횟수가 `max_miss_count`를 초과하면 트랙을 삭제한다.

```
miss_count > max_miss_count → 트랙 삭제

예시 (max_miss_count=5):
  miss 0~5회 → 트랙 유지 (predict 위치로 예측)
  miss 6회   → 트랙 삭제
```

## 파라미터

| 파라미터 | 기본값 | 단위 | 설명 |
|---------|--------|------|------|
| `wheelbase` | 0.87 | m | T870 축간거리 (Bicycle Model) |
| `min_match_dist` | 0.1 | m | 동적 매칭 최소 거리 (저속/정지 시 하한) |
| `max_miss_count` | 5 | -- | 이 횟수 초과 시 트랙 삭제 |
| `input_topic` | /perception/raw_bboxes | -- | 입력 bbox 토픽 |
| `output_topic` | /perception/bboxes | -- | 출력 tracked bbox 토픽 |
| `control_topic` | /t870/control_command | -- | 제어 명령 토픽 |

config 경로: `lidar_launch/config/bbox_tracker/bbox_tracker_params.yaml`

## 디버그 시각화 (RViz2)

- `/tracker/debug/tracks`: 전체 트랙 포인트
  - 초록: 현재 프레임에서 검출됨 (miss_count = 0)
  - 빨강: 검출 안 됨, predict로 유지 중 (miss_count > 0)
- `/tracker/debug/predicted`: 예측 유지 트랙만 주황색 큐브로 표시
  - LiDAR 사각지대의 bbox가 어디에 있다고 추정하는지 확인 가능

## 소스 파일

| 파일 | 역할 |
|------|------|
| `include/bbox_tracker/bbox_tracker_node.hpp` | 노드 클래스 + Track 구조체 선언 |
| `src/bbox_tracker_node.cpp` | 전체 트래킹 로직 구현 |
| `config/bbox_tracker.yaml` | 로컬 파라미터 (참고용) |
| `launch/bbox_tracker.launch.py` | 노드 실행 (lidar_launch config 참조) |
