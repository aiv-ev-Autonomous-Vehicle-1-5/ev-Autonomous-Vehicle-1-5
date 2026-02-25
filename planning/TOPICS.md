# track_planning 토픽 인터페이스

## 전체 데이터 흐름

```
┌──────────────────┐     ┌───────────────────────────────────────────────┐     ┌──────────────┐
│   Perception     │     │            LocalPlannerNode (10Hz)            │     │  Controller  │
│ (카메라 + LiDAR) │     │                                               │     │  (제어기)    │
├──────────────────┤     │  parse → corridor → virtual → centerline     │     ├──────────────┤
│ /perception/     │────►│  → postprocess → safety → publish             │────►│ /planning/   │
│   lane_boundaries│     │                                               │     │   path       │
│   cones          │     │  Debug: corridor_left/right, centerline,      │     │   status     │
└──────────────────┘     │         virtual_used                          │     └──────────────┘
                         └───────────────────────────────────────────────┘
```

## Subscriptions (입력: 2개)

| 토픽 | 메시지 타입 | QoS | 설명 |
|------|-------------|-----|------|
| `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray` | Reliable, depth=1 | 카메라 기반 차선 경계 |
| `/perception/cones` | `track_msgs/ConeArray` | Reliable, depth=1 | LiDAR 기반 콘 위치 |

### 메시지 상세

**LaneBoundary** — 단일 차선 경계선
```
std_msgs/Header header
geometry_msgs/Point[] points    # 순서 정렬된 경계점 배열 (차량에서 가까운 점부터)
uint8 LEFT  = 0                 # 좌측 차선 경계 상수
uint8 RIGHT = 1                 # 우측 차선 경계 상수
uint8 side                      # 이 경계의 좌/우 구분
float32 confidence              # 검출 신뢰도 (0.0 ~ 1.0)
```

**Cone** — 단일 콘 (라바콘/교통콘)
```
geometry_msgs/Point position        # 콘 중심 좌표 (x, y, z) [m]
geometry_msgs/Vector3 dimensions    # 콘 크기 (width, depth, height) [m]
float32 confidence                  # 검출 신뢰도 (0.0 ~ 1.0)
int32 label                         # 클러스터 라벨 또는 클래스 ID
```

### 입력 파싱 규칙

- **차선**: `boundary.side == LEFT` → `lane_left`, `RIGHT` → `lane_right`
- **콘**: `cone.position.y >= 0` → `cone_left` (좌측), `< 0` → `cone_right` (우측)
- 콘이 차선보다 높은 우선순위 (Cone Priority)

## Publishers - Core (출력: 2개)

| 토픽 | 메시지 타입 | QoS | 설명 |
|------|-------------|-----|------|
| `/planning/path` | `nav_msgs/Path` | Reliable, depth=1 | 후처리 완료된 최종 경로 |
| `/planning/status` | `track_msgs/PlannerStatus` | Reliable, depth=1 | 플래너 상태 + 목표 속도 |

**PlannerStatus** — 플래너 상태 출력
```
std_msgs/Header header
uint8 OK          = 0    # 정상 — 경로 유효, 주행 가능
uint8 STOP        = 1    # 정지 — 경로 생성 실패
uint8 INFEASIBLE  = 2    # 실현불가 — 곡률 초과 (차량 조향 한계)
uint8 STALE       = 3    # 만료 — 입력 데이터 지연 (perception_ms 초과)
uint8 status              # 현재 상태
string reason             # 상태 설명 문자열
```

### reason 필드 값

| reason | 상태 | 의미 |
|--------|------|------|
| `"ok"` | OK | 정상 주행 가능 |
| `"input_stale"` | STALE | 입력 데이터 타임아웃 초과 |
| `"no_valid_path"` | STOP | 경로 생성 실패 |
| `"curvature_exceeds_r_min"` | INFEASIBLE | 경로 곡률이 최소 회전 반경 초과 |

## Publishers - Debug (디버그: 4개)

RViz2 시각화 전용. **구독자가 없으면 메시지 생성을 건너뜀** (Lazy Publishing).

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/planning/debug/corridor_left` | `nav_msgs/Path` | 좌측 코리더 경계 (CorridorBuilder 출력) |
| `/planning/debug/corridor_right` | `nav_msgs/Path` | 우측 코리더 경계 |
| `/planning/debug/centerline` | `nav_msgs/Path` | DTR 센터라인 (후처리 전) |
| `/planning/debug/virtual_used` | `std_msgs/Bool` | 이번 프레임에서 가상 경계 사용 여부 |

## Stale Policy (데이터 만료 정책)

- **판정 기준**: lanes 또는 cones 중 **하나 이상**이 `perception_ms` (기본 300ms) 이내
- **OR 조건**: 카메라 또는 LiDAR 둘 중 하나만 살아있어도 파이프라인 진행
- **둘 다 만료 시**: STALE 상태 → `target_speed = 0` → 차량 정지
- **타임스탬프**: 메시지 수신 시 `now()`로 기록, `check_stale()`에서 차이 계산

## 좌표계

- **프레임**: `base_link` (ego 차량 기준)
- **방향**: 전방 +x, 좌측 +y, 위쪽 +z
- **단위**: 미터 (m), 라디안 (rad)
- **heading**: atan2(y, x), +x = 0°, 반시계 방향 양수
