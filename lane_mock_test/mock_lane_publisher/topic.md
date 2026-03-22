# mock_lane_publisher 패키지 토픽 정리

## 구독 토픽
없음

## 발행 토픽
| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/lane_boundaries` | `ev_msgs/msg/LaneBoundaryArray` | BestEffort, depth=1 | 테스트용 직선 차선 경계 (base_link 프레임) |
