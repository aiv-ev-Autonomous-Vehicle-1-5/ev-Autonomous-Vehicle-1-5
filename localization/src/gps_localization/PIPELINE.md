# gps_localization 패키지 파이프라인

## 패키지 정보
- **Package**: gps_localization
- **Description**: GPS -> UTM 변환 기반 위치추정 및 웨이포인트 관리 패키지
- **ROS 2 Humble, C++17**

---

## 노드

### 1. utm_localizer_node

GPS 위치추정 메인 노드. GPS 원시 데이터를 수신하여 UTM 로컬 좌표로 변환하고, 차량 기준 경로 마커를 발행한다.

#### 파이프라인

```
GPS /fix 수신
  -> 공분산 필터 (covariance_threshold 초과 시 무시)
  -> UTM 변환 (WGS84 Zone 52)
  -> 원점 설정 (최초 유효 GPS 위치를 원점으로 설정)
  -> 로컬 좌표 변환 (UTM -> Map 원점 상대)
  -> GPS 점프 필터 (jump_threshold 초과 시 무시)
  -> 이동 방향 yaw 추정
  -> base_link 프레임 변환
  -> Marker POINTS 발행 (/local_path)
```

#### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `jump_threshold` | double | 2.0 | GPS 점프 필터 임계값 (m) |
| `covariance_threshold` | double | 0.5 | 공분산 필터 임계값 |
| `lookahead_m` | double | 6.0 | 경로 전방 주시 거리 (m) |
| `path_points` | int | 30 | 경로 포인트 수 |

---

### 2. waypoint_recorder_node

GPS 웨이포인트를 CSV 파일로 기록하는 노드.

#### 파이프라인

```
GPS /fix 수신
  -> 공분산 필터 (covariance_threshold 초과 시 무시)
  -> UTM 변환 (WGS84 Zone 52)
  -> 거리 검사 (min_distance 미만이면 무시)
  -> CSV 기록 (index, utm_x, utm_y)
```

#### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `record_interval` | double | 0.5 | 기록 간격 (s) |
| `covariance_threshold` | double | 0.5 | 공분산 필터 임계값 |
| `min_distance` | double | 0.3 | 최소 기록 거리 (m) |

---

## 좌표 프레임

```
UTM (WGS84 Zone 52)
  -> Map (원점 상대 로컬 좌표)
  -> base_link (차량 기준 상대 좌표)
```
