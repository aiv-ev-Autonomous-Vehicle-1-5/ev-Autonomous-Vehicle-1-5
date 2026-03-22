# lane_seg_msgs 파이프라인

## 패키지 정보
- **Package**: lane_seg_msgs
- **Description**: 차선 인식 노드 출력용 커스텀 메시지 정의 패키지

## 메시지 정의

### LaneCoords.msg
| 필드 | 타입 | 설명 |
|---|---|---|
| `line_x` | `float32[]` | 종방향 좌표 (전방 거리, m) |
| `line_y` | `float32[]` | 횡방향 좌표 (좌/우 거리, m, +좌/-우) |

## 사용처
- `yolo_seg_node` -> `/lane_coordinates` 토픽
