# EV 자율주행 시스템

## 빌드

```bash
cd ~/ev_ws
colcon build --symlink-install
source install/setup.bash
```

> 빌드 중 PC가 멈출 수 있음 (메모리 부족). 재부팅 후 다시 시도.
> 병렬 제한: `colcon build --symlink-install --parallel-workers 2`

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
