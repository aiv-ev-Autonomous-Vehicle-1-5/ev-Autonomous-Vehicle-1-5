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

### 방법 1 — tmuxp로 한 번에 실행 (권장)

```bash
tmuxp load tmux_config/ev.yaml
```

이 한 줄이면 tmux 세션이 생성되고, source + 각 노드 launch가 자동으로 실행됨.

### 방법 2 — 수동 실행

각 터미널에서 `source ~/ev_ws/install/setup.bash` 후 순서대로 실행:

```bash
# 터미널 1 — Gazebo 시뮬레이션 (가상 센서 발행)
ros2 launch carsa_gazebo empty.launch.py

# 터미널 2 — LiDAR 파이프라인 (지면분리 → 클러스터링 → BBox)
ros2 launch lidar_launch make_bbox.launch.py

# 터미널 3 — 경로 계획 (Chainer → Costmap → A* → Path)
ros2 launch chaining_costmap_ver chaining_costmap_ver.launch.py
```

---

## tmux / tmuxp 사용법

### tmuxp란?

tmux 세션을 YAML 파일로 정의해서 한 번에 띄워주는 도구.
`tmux_config/ev.yaml`에 세션 구성이 정의되어 있음.

### tmuxp 명령어

| 명령어 | 설명 |
|--------|------|
| `tmuxp load tmux_config/ev.yaml` | YAML 설정대로 세션 생성 + 자동 실행 |
| `tmuxp load -d tmux_config/ev.yaml` | 백그라운드로 세션 생성 (detached) |
| `tmuxp freeze ev` | 현재 실행 중인 `ev` 세션을 YAML로 내보내기 |

### tmux 기본 조작

> prefix가 `F2`로 변경되어 있음 (`~/.tmux.conf` 참고). 마우스도 활성화되어 있어서 pane 클릭/스크롤 가능.

#### 세션 관리

| 단축키 / 명령어 | 설명 |
|------------------|------|
| `tmux ls` | 실행 중인 세션 목록 |
| `tmux attach -t ev` | `ev` 세션에 접속 |
| `F2` → `d` | 세션에서 빠져나오기 (detach, 세션은 살아있음) |
| `tmux kill-session -t ev` | `ev` 세션 종료 |

#### 윈도우 관리 (`F2` prefix)

| 단축키 | 설명 |
|--------|------|
| `F2` → `c` | 새 윈도우 생성 |
| `F2` → `l` | 윈도우 목록 선택 |
| `F2` → `n` | 현재 윈도우 이름 변경 |
| `F2` → `←` / `→` | 이전/다음 윈도우 |
| `F2` → `↑` | 마지막 윈도우로 점프 |
| `F2` → `w` | 현재 윈도우 종료 (확인 물어봄) |
| `F2` → `f` | 윈도우 검색 |

#### pane 조작 (`F3` prefix)

| 단축키 | 설명 |
|--------|------|
| `F3` → `방향키` | 인접 pane으로 이동 |
| `F3` → `z` | 현재 pane 전체화면 토글 (zoom) |
| `F3` → `e` | 좌우 분할 (vertical split) |
| `F3` → `o` | 상하 분할 (horizontal split) |
| `F3` → `w` | 현재 pane 종료 |
| `F3` → `1` | tiled 레이아웃 |
| `F3` → `2` | even-horizontal 레이아웃 |
| `F3` → `3` | even-vertical 레이아웃 |
| `F3` → `Esc` | pane 모드 취소 |

#### 스크롤

마우스 스크롤로 바로 가능 (mouse on 설정). 또는:

| 단축키 | 설명 |
|--------|------|
| `F2` → `[` | 스크롤 모드 진입 (방향키/PgUp/PgDn) |
| `q` | 스크롤 모드 나가기 |

### ev.yaml 구조 설명

```yaml
session_name: ev                          # 세션 이름
start_directory: /home/aiv/ev-Autonomous-Vehicle-1-5

shell_command_before:                     # 모든 pane에 공통 실행
  - source /opt/ros/humble/setup.bash
  - source /home/aiv/ev-Autonomous-Vehicle-1-5/install/setup.bash

windows:
  - window_name: main
    layout: even-horizontal               # pane 균등 분할
    panes:
      - ros2 launch carsa_gazebo empty.launch.py           # pane 0: Gazebo
      - ros2 launch lidar_launch make_bbox.launch.py       # pane 1: Perception
      - ros2 launch chaining_costmap_ver chaining_costmap_ver.launch.py  # pane 2: Planning
```

### pane 추가하는 법

`ev.yaml`의 `panes` 리스트에 항목을 추가하면 됨:

```yaml
    panes:
      - ros2 launch carsa_gazebo empty.launch.py
      - ros2 launch lidar_launch make_bbox.launch.py
      - ros2 launch chaining_costmap_ver chaining_costmap_ver.launch.py
      - ros2 launch pp_controller_cpp pure_pursuit_relative.launch.py   # 새 pane 추가
```

### 자주 쓰는 시나리오

```bash
# 세션 시작
tmuxp load tmux_config/ev.yaml

# 작업 중 잠깐 빠져나오기
F2 → d

# 다시 돌아오기
tmux attach -t ev

# 특정 pane의 로그 확인 (zoom)
F3 → 방향키로 이동 → F3 → z

# 세션 완전 종료
tmux kill-session -t ev
```

## 데이터 흐름

```
[Gazebo]  velodyne_points
    → [Perception]  Patchwork++ → DBSCAN → MakeBBox → /perception/bboxes
    → [Planning]    Chainer → Costmap → A* → Postprocess → /planning/path
```
