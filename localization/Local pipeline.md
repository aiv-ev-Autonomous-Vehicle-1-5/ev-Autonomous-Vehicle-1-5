# gps_localization (UTM Localizer) README
## 1. 이 패키지가 하는 일

`gps_localization`의 `utm_localizer` 노드는 GNSS에서 들어오는 `/fix(NavSatFix)`를 받아서
* 위도/경도(lat/lon) → UTM 변환
* 최초 1회 origin(기준점) 설정
* UTM 좌표를 origin 기준으로 local(map) 좌표(m)로 변환
* 연속된 위치 변화로 yaw(헤딩 각도) 추정 후 quaternion으로 변환
* 결과를 /local_pose로 publish

즉, planning이 GPS 모드 path를 만들 때 필요한 “현재 위치/헤딩”을 제공하는 역할이다.

## 2. 입출력 토픽 정리
Input
* `/fix (sensor_msgs/msg/NavSatFix)`
  - 실제 GNSS 드라이버가 `/ublox_gps_node/fix`로 publish한다면, 노드 코드/런치에서 토픽명을 맞춰야 함(현재 버전은 `/fix` 구독)

Output
* `/local_pose (geometry_msgs/msg/PoseStamped)`
  - `header.frame_id = "map"`
  - `pose.position.x/y` : origin 기준 local 좌표(m)
  - `pose.orientation` : GPS 연속 이동 방향 기반 yaw를 quaternion으로 반영 (처음 1~2프레임은 yaw가 안정화되기 전이라 기본값일 수 있음)

## 3. 동작 파이프라인(내부 로직)

1. `/fix` 수신
2. (옵션) 공분산 필터
`position_covariance_type != UNKNOWN`이면 cov_xx, cov_yy 기반으로 필터링
3. lat/lon → UTM 변환
4. origin 미설정이면 최초 1회 origin(utm_x, utm_y) 저장
5. local 좌표: local = utm - origin
6. 점프 필터: 이전 local과의 거리 > jump_threshold면 무시
7. yaw 추정:
  - yaw = atan2(dy, dx) (연속 local 이동 벡터)
  - 너무 작은 이동이면 yaw 업데이트 freeze
  - yaw 급점프는 outlier로 무시
8. `/local_pose publish`

## 4. 빌드

워크스페이스 루트(예: `.../localization`)에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gps_localization
source install/setup.bash
```

## 5. 실행
### 5.1 기본 실행(현재 코드가 /fix를 구독)
```bash
ros2 run gps_localization utm_localizer_node --ros-args -r __node:=utm_localizer
```

### 5.2 ublox 토픽을 그대로 쓰고 싶다면

현재 코드가 `/fix`를 구독하므로, 다음 중 하나로 맞춘다.

방법 A) ublox 토픽을 /fix로 리맵
```bash
# ublox가 /ublox_gps_node/fix 로 publish 한다고 가정
# (해당 노드 실행 시)
--ros-args -r /ublox_gps_node/fix:=/fix
```

방법 B) utm_localizer 코드에서 subscribe 토픽을 `/ublox_gps_node/fix`로 변경
* `create_subscription(... "/fix" ...)` 부분을 `/ublox_gps_node/fix`로 변경 후 재빌드

팀 내 표준은 “토픽명은 파라미터로 빼기”가 가장 좋지만, 현재 버전은 코드 고정형이다.

## 6. 빠른 테스트(안테나 없이도 가능): 가짜 /fix 발행
### 6.1 utm_localizer 실행

```bash
ros2 run gps_localization utm_localizer_node --ros-args -r __node:=utm_localizer
6.2 /fix 3연타 (약 0.6m 간격 이동 → yaw 추정 확인)
ros2 topic pub --once /fix sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 33.30334533, longitude: 126.29904780, altitude: 0.0, position_covariance: [0.04,0,0, 0,0.04,0, 0,0,0], position_covariance_type: 2}"
ros2 topic pub --once /fix sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 33.30335133, longitude: 126.29904780, altitude: 0.0, position_covariance: [0.04,0,0, 0,0.04,0, 0,0,0], position_covariance_type: 2}"
ros2 topic pub --once /fix sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 33.30335733, longitude: 126.29904780, altitude: 0.0, position_covariance: [0.04,0,0, 0,0.04,0, 0,0,0], position_covariance_type: 2}"
```

### 6.3 출력 확인

```bash
ros2 topic echo /local_pose --once
```

기대 결과
* `pose.position.x/y`가 0에서 조금씩 변함
* `pose.orientation.z/w`가 (0,0,0,1)이 아닌 값으로 나오면 yaw 추정 정상

## 7. Planning과의 연결(현재 단계)

현재 localization은 /local_pose(map)를 제공한다.
Planning은 이를 구독해서 GPS 모드 path를 만든다.

컨트롤 입력
* /planning/path (nav_msgs/Path, frame=base_link)

GPS 모드 path 생성 방식(권장)

Planning에서:
* `/local_pose(map)`로 현재 위치/헤딩을 받고
* waypoint(track.csv 등)에서 가까운 구간을 선택 후
* `base_link`로 변환해서 `/planning/path publish`

즉, localization은 “현재 pose 제공”, planning은 “최종 경로 생성” 역할로 분리한다.

```css
GPS 가능 구간      → GPS 기반 localization
GPS 불가 구간      → odom / dead reckoning / lidar-localization 기반 localization
                    ↓
            최종 fused pose 1개 생성
                    ↓
            TF: map → base_link publish
                    ↓
                 Planning
```

Localization이 연결 전체 파이프라인
```css
GPS
 ↓
Localization
 ↓
TF map → base_link
 ↓
Perception
 ↓
Planning
 ↓
Control
```
