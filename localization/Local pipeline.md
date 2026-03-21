# 전체 구성도

```
/ublox_gps_node/fix (sensor_msgs/NavSatFix)
          │
          ▼
gps_localization / utm_localizer_node
  └─ /local_path (visualization_msgs/Marker POINTS, frame=base_link)
     전체 waypoint를 차량 헤딩 기준 상대 좌표로 변환
          │
          ▼
       planning
```

# 토픽/프레임 명세

utm_localizer (gps_localization)

* Input:
  - /ublox_gps_node/fix (sensor_msgs/NavSatFix) — 파라미터 `fix_topic`으로 변경 가능
* Output:
  - /local_path (visualization_msgs/Marker, type=POINTS)
    + frame: base_link
    + 전체 waypoint를 차량 헤딩 기준 상대 좌표(x, y)로 변환하여 points 배열로 발행
* Parameters:
  - `fix_topic` (string, 기본값: `/ublox_gps_node/fix`) — 구독할 GNSS 토픽명
  - `waypoint_file` (string, 기본값: `""`) — "index,utm_x,utm_y" 형식 CSV 파일 경로 (예: RDDF/sejong_playground.csv)
  - `jump_threshold` (double, 기본값: `2.0`) — GPS 튐 거리 임계값(m)
  - `covariance_threshold` (double, 기본값: `0.5`) — 공분산 기반 sigma_xy 임계값(m)

# 실행 방법

```bash
ros2 run gps_localization utm_localizer_node
```

# 동작 파이프라인(내부 로직)

1. `/fix` 수신
2. 공분산 필터: sigma_xy > `covariance_threshold` → 무시
3. lat/lon → UTM 변환
4. 최초 1회 origin 설정, CSV waypoint(UTM)를 local(map) 좌표로 1회 변환 캐싱 (waypoint_utm - origin)
5. local 좌표: local = utm - origin
6. 점프 필터: 이전 local과 거리 > `jump_threshold` → 무시
7. yaw 추정: atan2(dy, dx), 저속 시 freeze, 급점프 outlier 무시
8. `/local_path` 발행 (frame=base_link): 전체 waypoint를 현재 위치/헤딩 기준 상대 좌표로 변환

---

# waypoint_recorder (gps_localization)

GPS 좌표를 주기적으로 수신하여 UTM 변환 후 CSV 파일로 저장하는 노드.
기록된 CSV는 utm_localizer의 waypoint_file 파라미터로 바로 사용 가능.

```
/ublox_gps_node/fix (sensor_msgs/NavSatFix)
          │
          ▼
gps_localization / waypoint_recorder_node
  └─ output CSV (index,utm_x,utm_y)
```

* Input:
  - /ublox_gps_node/fix (sensor_msgs/NavSatFix) — 파라미터 `fix_topic`으로 변경 가능
* Output:
  - CSV 파일 (index,utm_x,utm_y) — utm_localizer와 동일 포맷
* Parameters:
  - `fix_topic` (string, 기본값: `/ublox_gps_node/fix`) — 구독할 GNSS 토픽명
  - `output_file` (string, 기본값: `""`) — 저장할 CSV 경로. 빈 문자열이면 `waypoints_YYYYMMDD_HHMMSS.csv` 자동 생성
  - `record_interval` (double, 기본값: `0.5`) — 기록 주기(s)
  - `covariance_threshold` (double, 기본값: `0.5`) — 공분산 기반 sigma_xy 임계값(m)
  - `min_distance` (double, 기본값: `0.3`) — 연속 기록 최소 거리(m), 정차 시 중복 방지

## 동작 파이프라인

1. `/fix` 수신
2. 공분산 필터: sigma_xy > `covariance_threshold` → 무시
3. lat/lon → UTM 변환, 최신 유효 좌표 캐싱
4. 타이머 주기(`record_interval`)마다:
   - 새 유효 fix가 있고
   - 이전 기록점과 `min_distance` 이상 떨어져 있으면
   - CSV에 `index,utm_x,utm_y` 한 줄 추가
5. 노드 종료 시 파일 닫기 및 총 기록 수 출력

## 실행 방법

```bash
# launch 파일로 실행
ros2 launch gps_localization waypoint_recorder.launch.py

# 직접 실행 (파라미터 지정)
ros2 run gps_localization waypoint_recorder_node --ros-args \
  -p output_file:=/home/aiv/ev-Autonomous-Vehicle-1-5/localization/RDDF/my_track.csv \
  -p record_interval:=0.5
```

---

# 빌드

```bash
colcon build --symlink-install --packages-select gps_localization
```
