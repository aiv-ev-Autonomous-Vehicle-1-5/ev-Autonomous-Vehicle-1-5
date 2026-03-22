# mock_lane_publisher 파이프라인

## 패키지 개요
- **패키지**: mock_lane_publisher
- **설명**: 하드코딩 직선 차선을 발행하는 테스트 노드 — 카메라 차선 인식 전 planning 테스트용
- **노드**: mock_lane_node (`MockLanePublisher`)

## 기능
고정 좌/우 직선 차선 경계를 주기적으로 발행합니다.

- **좌측 경계**: y = +lane_half_width, x ∈ [lane_x_start, lane_x_end]
- **우측 경계**: y = -lane_half_width, x ∈ [lane_x_start, lane_x_end]
- **포인트 순서**: 가까운 → 먼 (x 증가 방향)
- **프레임**: `base_link`

## 파라미터
| 파라미터 | 코드 기본값 | config 기본값 | 설명 |
|---|---|---|---|
| `lane_half_width` | `1.5` | `2.0` | 중앙에서 좌/우 오프셋 [m] |
| `lane_x_start` | `0.5` | `0.5` | 차선 시작 x [m] |
| `lane_x_end` | `5.0` | `5.0` | 차선 끝 x [m] |
| `point_spacing` | `0.5` | `0.5` | 점 간격 [m] |
| `publish_hz` | `10.0` | `10.0` | 발행 주기 [Hz] |

## 데이터 흐름
```
mock_lane_node
    │
    └──▶ /perception/lane_boundaries  (ev_msgs/msg/LaneBoundaryArray, BestEffort depth=1)
              │
              └──▶ planning 파이프라인 (chaining_costmap 등)
```

## 실행
```bash
ros2 launch mock_lane_publisher mock_lane_launch.py
```
