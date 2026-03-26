# bbox_tracker 패키지 토픽 정리

> **[DEPRECATED]** bbox_tracker는 현재 파이프라인에서 제거되었습니다.
> make_bbox가 `/perception/bboxes`로 직접 발행하고, planning이 이를 직접 구독합니다.

## 구독 토픽 (DEPRECATED)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/raw_bboxes` | `ev_msgs/msg/BBoxArray` | BestEffort, depth=10 | make_bbox 원시 검출 결과 (토픽 폐기됨) |
| `/t870/control_command` | `t870_msgs/msg/ControlCommand` | BestEffort, depth=10 | 차량 조향(rad) 및 속도(m/s) -- 자아운동 보정용. header에 센서 타임스탬프 포함 |

## 발행 토픽 (DEPRECATED)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/perception/bboxes` | `ev_msgs/msg/BBoxArray` | BestEffort, depth=10 | 트래킹된 바운딩 박스 (현재는 make_bbox가 직접 발행) |

## 디버그 토픽 (DEPRECATED)

| 토픽 이름 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `/tracker/debug/tracks` | `visualization_msgs/msg/Marker` (POINTS) | Reliable, depth=1 | 전체 트랙 (초록=감지, 빨강=예측) |
| `/tracker/debug/predicted` | `visualization_msgs/msg/Marker` (CUBE_LIST) | Reliable, depth=1 | 예측 전용 트랙 (주황 큐브) |
