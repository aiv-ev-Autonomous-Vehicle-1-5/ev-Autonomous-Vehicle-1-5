# track_planning Topic Interface

## Subscriptions (2)

| Topic | Message Type | Fields | Description |
|-------|-------------|--------|-------------|
| `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray` | `header`, `LaneBoundary[] boundaries` | 카메라 기반 차선 경계 |
| `/perception/cones` | `track_msgs/ConeArray` | `header`, `Cone[] cones` | LiDAR 기반 콘 위치 |

### Message Details

**LaneBoundary**
```
std_msgs/Header header
geometry_msgs/Point[] points    # 정렬된 경계점 배열
uint8 side                      # LEFT=0, RIGHT=1
float32 confidence              # 0.0 ~ 1.0
```

**Cone**
```
geometry_msgs/Point position        # 중심 좌표 (x, y, z)
geometry_msgs/Vector3 dimensions    # (width, depth, height)
float32 confidence                  # 0.0 ~ 1.0
int32 label                         # 클러스터 라벨
```

## Publishers - Core (2)

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/planning/path` | `nav_msgs/Path` | 후처리 완료된 최종 경로 |
| `/planning/status` | `track_msgs/PlannerStatus` | 플래너 상태 + 사유 |

**PlannerStatus**
```
std_msgs/Header header
uint8 status    # OK=0, STOP=1, INFEASIBLE=2, STALE=3
string reason   # 사유 코드 ("ok", "no_valid_path", "curvature_exceeds_r_min", "stale")
```

## Publishers - Debug (4)

RViz2 시각화용. 구독자가 없으면 메시지 생성을 건너뜀 (Lazy Publishing).

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/planning/debug/corridor_left` | `nav_msgs/Path` | 좌측 코리더 경계 |
| `/planning/debug/corridor_right` | `nav_msgs/Path` | 우측 코리더 경계 |
| `/planning/debug/centerline` | `nav_msgs/Path` | DTR 센터라인 |
| `/planning/debug/virtual_used` | `std_msgs/Bool` | 가상 경계 사용 여부 |

## Stale Policy

- **Perception**: lanes 또는 cones 중 하나 이상이 `perception_ms` (기본 300ms) 이내
- 둘 다 만료 시 STALE 상태 → 차량 정지
