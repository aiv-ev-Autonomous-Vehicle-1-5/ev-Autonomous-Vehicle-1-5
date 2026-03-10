# EV 자율주행 시스템

## 빌드
### 방법 1
    carsa_gazebo, perception, planning 등등 하위 패키지별로 각각 build 한 후에 setup.bash 모두 source해서 사용하는게 나중에 모듈별로 디버깅하기 편함. 그걸 권장. 그냥 간단하게 처음 받아봤을때 돌아가는지 테스트해보고 싶으면 방법2로 ㄱㄱ
### 방법 2

```bash
cd ~/ev_ws
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```
    그냥 빌드하면 PC가 멈추므로 다음 명령어로 빌드해야함. 
    병렬 제한: `colcon build --symlink-install --parallel-workers 2`

## 주의!
새 터미널 프로세스 실행할 때마다 
```bash 
source install/setup.bash 
```
안 하고 안 된다고 뇌절 ㄴㄴ


## 실행

각 터미널에서 `source ~/ev_ws/install/setup.bash` 후 순서대로 실행:

```bash
# 터미널 1 — Gazebo 시뮬레이션 (가상 센서 발행)
ros2 launch carsa_gazebo empty.launch.py

# 터미널 2 — LiDAR 파이프라인 (지면분리 → 클러스터링 → BBox)
ros2 launch lidar_launch make_bbox.launch.py

# 터미널 3 — 경로 계획 (Chainer → Costmap → A* → Path)
ros2 launch chaining_costmap_ver chaining_costmap_ver.launch.py
```

## 데이터 흐름

```
[Gazebo]  velodyne_points
    → [Perception]  Patchwork++ → DBSCAN → MakeBBox → /perception/bboxes
    → [Planning]    Chainer → Costmap → A* → Postprocess → /planning/path
```
