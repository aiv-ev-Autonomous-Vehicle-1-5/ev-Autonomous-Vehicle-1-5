# track_planning 파이프라인 상세 문서

> **패키지**: `track_planning`
> **노드**: `LocalPlannerNode` (10Hz, ComposableNode)
> **좌표계**: ego-centric `base_link` (전방 +x, 좌측 +y)

---

## 목차

1. [아키텍처 개요](#1-아키텍처-개요)
2. [파일 구조](#2-파일-구조)
3. [데이터 타입 정의](#3-데이터-타입-정의)
4. [파이프라인 전체 흐름](#4-파이프라인-전체-흐름)
5. [Stage 0: Stale 게이트](#5-stage-0-stale-게이트)
6. [Stage 1: 입력 파싱](#6-stage-1-입력-파싱)
7. [Stage 2: Corridor Build (Greedy Chaining)](#7-stage-2-corridor-build)
8. [Stage 3: Virtual Boundary (가상 경계)](#8-stage-3-virtual-boundary)
9. [Stage 4: Centerline Build (DTR 외심)](#9-stage-4-centerline-build)
10. [Stage 5: Raw Path 생성](#10-stage-5-raw-path-생성)
11. [Stage 6: Postprocess (후처리)](#11-stage-6-postprocess)
12. [Stage 7: Safety Check (안전 검사)](#12-stage-7-safety-check)
13. [Stage 8: Publish (퍼블리시)](#13-stage-8-publish)
14. [기하 유틸리티 (geometry.hpp)](#14-기하-유틸리티)
15. [파라미터 레퍼런스](#15-파라미터-레퍼런스)
16. [토픽 인터페이스](#16-토픽-인터페이스)
17. [빌드 구조](#17-빌드-구조)

---

## 1. 아키텍처 개요

```
┌─────────────────────────────────────────────────────────────────────┐
│                    LocalPlannerNode (10Hz Timer)                     │
│                                                                     │
│  ┌──────────┐   ┌───────────┐   ┌─────────────┐   ┌─────────────┐  │
│  │  Stale   │──▶│ Corridor  │──▶│  Virtual    │──▶│ Centerline  │  │
│  │  Gate    │   │  Builder  │   │  Boundary   │   │  Builder    │  │
│  │ (Stage0) │   │ (Stage2)  │   │  (Stage3)   │   │  (Stage4)   │  │
│  └──────────┘   └───────────┘   └─────────────┘   └─────────────┘  │
│       │                                                  │          │
│       │              ┌──────────────┐   ┌────────────┐   │          │
│       │              │ Postprocess  │◀──│  Raw Path  │◀──┘          │
│       │              │  (Stage6)    │   │  (Stage5)  │              │
│       │              └──────┬───────┘   └────────────┘              │
│       │                     ▼                                       │
│       │              ┌──────────────┐   ┌────────────┐              │
│       │              │   Safety     │──▶│  Publish   │              │
│       │              │   Check      │   │  (Stage8)  │              │
│       │              │  (Stage7)    │   └────────────┘              │
│       │              └──────────────┘                                │
│       │                                                             │
│       └──── STALE 시 ──▶ status=STALE 즉시 퍼블리시 후 return        │
└─────────────────────────────────────────────────────────────────────┘

입력 토픽:
  /perception/lane_boundaries  (LaneBoundaryArray)  ← 카메라
  /perception/cones            (ConeArray)           ← LiDAR

출력 토픽:
  /planning/path               (nav_msgs/Path)       → 컨트롤러
  /planning/status             (PlannerStatus)        → 컨트롤러
```

**설계 원칙:**
- **단일 책임 원칙(SRP)**: 각 모듈(CorridorBuilder, VirtualBoundary, CenterlineBuilder, PathPostprocessor)이 한 가지 역할만 담당
- **ComposableNode 패턴**: `rclcpp_components`로 동적 로드, `use_intra_process_comms: True`로 Zero-copy 통신
- **UniquePtr 콜백**: 구독 시 `std::move`로 소유권 이전 → 메모리 복사 없음

---

## 2. 파일 구조

```
track_planning/
├── CMakeLists.txt                      # 빌드 설정 (3개 라이브러리)
├── package.xml                         # ROS 2 패키지 메타데이터
├── config/
│   └── planning.yaml                   # 전체 파라미터 정의
├── launch/
│   └── planning.launch.py              # ComposableNodeContainer launch
├── include/track_planning/
│   ├── common/
│   │   ├── types.hpp                   # Point2D, CorridorPolylines, PlannerState 등
│   │   ├── params.hpp                  # PlanningParams 구조체 + YAML 로딩
│   │   ├── geometry.hpp                # 2D 기하 유틸 (header-only)
│   │   └── debug_publish.hpp           # RViz2 메시지 변환 (header-only)
│   ├── corridor/
│   │   ├── corridor_builder.hpp        # Greedy Chaining 경계 빌더
│   │   ├── virtual_boundary.hpp        # 가상 경계 생성
│   │   └── centerline_builder.hpp      # DTR 외심 센터라인
│   ├── nodes/
│   │   └── local_planner_node.hpp      # 메인 노드 클래스
│   ├── postprocess/
│   │   └── path_postprocessor.hpp      # 경로 후처리
│   ├── safety/
│   │   └── safety_checker.hpp          # 안전 검사 (header-only)
│   └── third_party/CDT/               # Constrained Delaunay Triangulation
└── src/
    ├── nodes/
    │   └── local_planner_node.cpp      # 메인 파이프라인 구현
    ├── corridor/
    │   ├── corridor_builder.cpp        # Greedy Chaining 구현
    │   ├── virtual_boundary.cpp        # 가상 경계 구현
    │   └── centerline_builder.cpp      # DTR 센터라인 구현
    └── postprocess/
        └── path_postprocessor.cpp      # 후처리 구현
```

---

## 3. 데이터 타입 정의

> **파일**: `include/track_planning/common/types.hpp`

### 기본 타입

```cpp
struct Point2D {
  double x = 0.0;   // 전방(+) / 후방(-)
  double y = 0.0;   // 좌측(+) / 우측(-)
};
```

### 파이프라인 중간 결과물

| 구조체 | 생성 Stage | 필드 | 설명 |
|--------|-----------|------|------|
| `CorridorPolylines` | Stage 2 | `left`, `right`, `left_ok`, `right_ok` | 좌/우 경계 폴리라인 + 유효성 |
| `VirtualBoundaryResult` | Stage 3 | `boundary`, `success` | 가상 경계 폴리라인 |
| `CenterlineResult` | Stage 4 | `center`, `valid` | 중앙선 점 배열 |
| `PostprocessResult` | Stage 6 | `path`, `yaw`, `valid` | 최종 경로 + heading 각도 |

### 플래너 상태

```cpp
enum class PlannerState : uint8_t {
  OK          = 0,   // 정상 — 경로 + target_speed 출력
  STOP        = 1,   // 정지 — 경로 생성 실패
  INFEASIBLE  = 2,   // 실현불가 — 곡률 초과
  STALE       = 3    // 입력 데이터 지연/누락
};
```

---

## 4. 파이프라인 전체 흐름

> **파일**: `src/nodes/local_planner_node.cpp` → `on_timer()`

```
on_timer() [매 100ms = 10Hz 호출]
│
├─ (0) Stale Gate ──── 입력 만료? ──YES──▶ STALE 퍼블리시 → return
│                          │NO
├─ (1) Input Parse ─── ego=(0,0), heading=(1,0) 고정
│                       parse_lanes() → lane_left, lane_right
│                       parse_cones() → cone_left, cone_right
│
├─ (2) Corridor Build ─ corridor_builder_.build(input, params)
│                        → CorridorPolylines {left, right, left_ok, right_ok}
│
├─ (3) Virtual Boundary
│   ├─ both_ok? → w_hat_ = compute_median_width(left, right)
│   └─ one_side_ok? → virtual_boundary_.generate(visible, ...)
│                      → corridor의 빈 쪽을 가상 경계로 채움
│
├─ (4) Centerline Build ─ centerline_builder_.build(left, right, ...)
│                          → CenterlineResult {center, valid}
│
├─ (5) Raw Path ──── centerline → raw_path (직접 사용)
│
├─ (6) Postprocess ─ postprocessor_.process(raw_path, ...)
│                     prune → smooth → resample → yaw
│                     → PostprocessResult {path, yaw, valid}
│
├─ (7) Safety Check ─ safety_checker::check(pp_result, params)
│                      곡률 검사 → 속도 제한 계산
│                      → SafetyResult {state, target_speed, max_curvature}
│
└─ (8) Publish ──── /planning/path (최종 경로)
                    /planning/status (상태 + 이유)
                    /planning/debug/* (디버그, lazy)
```

---

## 5. Stage 0: Stale 게이트

> **파일**: `src/nodes/local_planner_node.cpp` → `check_stale()`

**목적**: 입력 데이터가 너무 오래되었으면 파이프라인 실행을 중단하고 즉시 STALE 상태를 퍼블리시한다.

**로직**:

```cpp
bool LocalPlannerNode::check_stale() const {
  const auto t = now();
  bool have_perception = false;

  if (last_lanes_) {
    double dt = (t - stamp_lanes_).nanoseconds() * 1e-6;  // ms 변환
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  if (last_cones_) {
    double dt = (t - stamp_cones_).nanoseconds() * 1e-6;
    if (dt <= params_.timeouts.perception_ms) have_perception = true;
  }
  return !have_perception;  // true = stale
}
```

**핵심 포인트:**
- lanes **또는** cones 중 **하나라도** 타임아웃 이내면 → stale이 아님 (OR 조건)
- 카메라 또는 LiDAR 둘 중 하나만 살아있어도 진행 가능
- 타임스탬프 초기값 `(0, 0, RCL_ROS_TIME)` → 수신 전에는 항상 stale

**Stale 시 동작** (`on_timer` 상단):

```cpp
if (input_stale) {
  auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
  status_msg->status = track_msgs::msg::PlannerStatus::STALE;
  status_msg->reason = "input_stale";
  pub_status_->publish(std::move(status_msg));
  return;  // 나머지 파이프라인 모두 스킵
}
```

**파라미터**: `timeouts.perception_ms` (기본 300ms)

---

## 6. Stage 1: 입력 파싱

> **파일**: `src/nodes/local_planner_node.cpp` → `parse_lanes()`, `parse_cones()`

### ego 상태 (고정)

센서 데이터가 이미 `base_link` 기준이므로:

```cpp
const Point2D ego_pos{0.0, 0.0};       // 항상 원점
const Point2D ego_heading{1.0, 0.0};   // 항상 +x 전방
```

### 차선 파싱 (`parse_lanes`)

```cpp
void LocalPlannerNode::parse_lanes(
  std::vector<Point2D> & left, std::vector<Point2D> & right) const
{
  if (!last_lanes_) return;
  for (const auto & b : last_lanes_->boundaries) {
    // side 필드로 좌/우 구분
    auto & target = (b.side == track_msgs::msg::LaneBoundary::LEFT) ? left : right;
    for (const auto & p : b.points) {
      target.push_back({p.x, p.y});
    }
  }
}
```

- 입력: `track_msgs::msg::LaneBoundaryArray` (`/perception/lane_boundaries`)
- `b.side == LEFT` → `lane_left`, 그 외 → `lane_right`

### 콘 파싱 (`parse_cones`)

```cpp
void LocalPlannerNode::parse_cones(
  std::vector<Point2D> & cone_left, std::vector<Point2D> & cone_right) const
{
  if (!last_cones_) return;
  for (const auto & c : last_cones_->cones) {
    Point2D pt{c.position.x, c.position.y};
    if (c.position.y >= 0.0) cone_left.push_back(pt);   // y >= 0: 좌측
    else                     cone_right.push_back(pt);   // y < 0: 우측
  }
}
```

- 입력: `track_msgs::msg::ConeArray` (`/perception/cones`)
- 좌/우 구분: **y 좌표 부호** (차량 좌표계에서 y > 0 = 좌측)

---

## 7. Stage 2: Corridor Build

> **파일**: `src/corridor/corridor_builder.cpp`, `include/track_planning/corridor/corridor_builder.hpp`

### 개요

Perception이 제공하는 차선점 + 콘 점을 **Greedy Chaining** 알고리즘으로 좌/우 경계 폴리라인으로 변환한다.

### 알고리즘 흐름

```
build(input, params)
│
├─ determine_reference(input)
│   └─ return {ego_pos=(0,0), ego_heading=(1,0)}
│
├─ build_one_side(lane_left, cone_left, c_end, t_end, ego, p)  → left
│   ├─ find_seed() → 시작점 탐색 (콘 우선)
│   └─ Greedy Loop:
│       ├─ 미사용 후보 수집 (used[] 플래그)
│       ├─ filter_candidates() → s/d 좌표계 필터링
│       ├─ Cone Priority: 콘 후보 있으면 콘만 사용
│       ├─ score_and_select() → 가중 점수 최적 선택
│       └─ chain에 추가, x > x_max이면 종료
│
├─ build_one_side(lane_right, cone_right, ...)  → right
│
└─ return CorridorPolylines {left, right, left_ok, right_ok}
```

### 7.1 참조점 결정 (`determine_reference`)

```cpp
std::pair<Point2D, Point2D> CorridorBuilder::determine_reference(const Input & in) {
  return {in.ego_pos, in.ego_heading};  // 항상 (0,0) + (1,0)
}
```

- 이전 프레임 의존을 완전 제거 → Cold Start 안전

### 7.2 Seed 탐색 (`find_seed`)

**적응형 반경 + 점수 기반** 시작점 선택:

1. 반경을 `r_seed_step`(1m)부터 `r_seed`(5m)까지 단계적으로 확장
2. 현재 반경 이내 + `forward_range`(60도) 이내인 점을 후보로 수집
3. 점수 계산:

```
score = w_dist * (거리 / 반경) + w_center * (1 / (1 + 횡편차))
```

4. **콘 우선(Cone Priority)**: 콘 seed가 있으면 차선 seed보다 항상 우선

```cpp
bool cone_found = find_best(cone_pts, cone_seed, cone_idx);
bool lane_found = find_best(lane_pts, lane_seed, lane_idx);

if (cone_found) { seed_out = cone_seed; seed_source = 0; return true; }
if (lane_found) { seed_out = lane_seed; seed_source = 1; return true; }
return false;
```

### 7.3 후보 필터링 (`filter_candidates`)

**s/d 좌표계**: 참조 접선(`t_end`) 기준으로 전방/횡방향 분해

```
diff = pt - c_end

s = dot(diff, t_end)          → 전방 진행거리 (양수 = 전방)
d = |cross(t_end, diff)|      → 횡방향 거리
```

**필터 조건** (모두 만족):

| 조건 | 수식 | 파라미터 | 기본값 |
|------|------|---------|--------|
| 최소 전방 | `s > s_min` | `corridor.filter.s_min` | 0.05m |
| 최대 전방 | `s < s_max` | `corridor.filter.s_max` | 2.0m |
| 최대 횡방향 | `d < d_max` | `corridor.filter.d_max` | 2.0m |
| 원형 반경 | `dist(pt, p_k) < r_search` | `corridor.filter.r_search` | 2.5m |

### 7.4 점수 산정 (`score_and_select`)

```
score = w_s * norm_s       (전방 진행 보상, +)
      - w_d * norm_d       (횡방향 편차 패널티, -)
      - w_a * norm_a       (방향 편차 패널티, -)
      - w_p * norm_p       (예측 오차 패널티, -)
```

각 항 정규화 [0, 1]:

| 항 | 수식 | 의미 |
|----|------|------|
| `norm_s` | `clamp(s / s_max)` | 더 앞에 있을수록 높은 보상 |
| `norm_d` | `clamp(d / d_max)` | 경계선 위에 가까울수록 낮은 패널티 |
| `norm_a` | `clamp(Δθ / θ_max)` | 현재 접선과 같은 방향일수록 낮은 패널티 |
| `norm_p` | `clamp(pred_err / r_search)` | 예측 위치에 가까울수록 낮은 패널티 |

**예측점**: `p_pred = p_k + step_pred * t_k` (현재 접선 방향으로 0.5m 이동)

**Top-K 최적화**: `corridor.topk.enable = true`시 d 기준 상위 K개만 상세 평가

### 7.5 Greedy Chaining 루프 (`build_one_side`)

```cpp
while (chain.size() < max_pts) {
  // 1. 로컬 접선 계산
  Point2D t_k = (chain.size() >= 2)
    ? regress_tangent(chain, n_reg)  // 마지막 n_reg개 점으로 회귀
    : t_end;                          // 첫 반복: 초기 접선

  // 2. 미사용 후보 수집 (used[] == false)
  // 3. filter_candidates()
  // 4. 콘 우선: cone_filtered 비어있지 않으면 콘 pool 사용
  // 5. score_and_select() → best
  // 6. chain.push_back(pool[best]), used[orig_idx] = true
  // 7. chain.back().x > x_max → break (ROI 초과)
}
```

**접선 회귀** (`regress_tangent`): 마지막 N개 세그먼트의 단위 방향벡터를 합산 후 정규화 → 노이즈에 강건한 평균 방향

---

## 8. Stage 3: Virtual Boundary

> **파일**: `src/corridor/virtual_boundary.cpp`, `include/track_planning/corridor/virtual_boundary.hpp`

### 목적

한쪽 경계만 보일 때, 추정 차로 폭(`w_hat`)으로 반대쪽 가상 경계를 생성한다.

### w_hat (추정 차로 폭) 관리

```cpp
// on_timer() 내부
if (both_ok) {
  w_hat_ = compute_median_width(corridor.left, corridor.right);
  // 양쪽 다 보일 때 직접 계산하여 갱신
}
// 한쪽만 보일 때 → w_hat_ 그대로 유지 (이전 값 또는 초기값 1.5m)
```

### 알고리즘

```
generate(visible, visible_is_left, w_hat, params)
│
├─ [Step 1] 최소 요건 확인
│   ├─ visible.size() < 2 → 실패
│   └─ w_hat < vehicle.width + safety.margin → 실패
│
├─ [Step 2] 접선 계산
│   └─ tangents = polyline_tangents(visible)
│
├─ [Step 3] 법선 방향 부호 결정
│   ├─ visible이 좌측 → sgn = -1 (가상은 우측 방향)
│   └─ visible이 우측 → sgn = +1 (가상은 좌측 방향)
│
├─ [Step 4] 가상점 생성
│   ├─ n = rotate90(tangent[i])          → 좌측 법선 벡터
│   ├─ p_virtual = visible[i] + sgn * w_hat * n
│   └─ ROI 밖이면 해당 점 스킵 (부분 경계도 허용)
│
└─ [Step 5] boundary.size() >= 2 → success = true
```

**기하학적 직관**:

```
rotate90(t) = (-t.y, t.x)   → CCW 90도 회전 = 좌측 법선

visible이 좌측(LEFT)일 때:
  접선 t →  법선 n ↑(좌측)
  가상은 우측이므로 → 법선 반대 → sgn = -1
  p_virtual = visible[i] - w_hat * n   (우측으로 이동)

visible이 우측(RIGHT)일 때:
  sgn = +1
  p_virtual = visible[i] + w_hat * n   (좌측으로 이동)
```

### on_timer에서의 가상 경계 적용

```cpp
if (!both_ok && one_side_ok) {
  visible_is_left = corridor.left_ok;
  auto virt_result = virtual_boundary_.generate(visible, visible_is_left, w_hat_, params_);

  if (virt_result.success) {
    virtual_used = true;
    // 빈 쪽을 가상 경계로 대체
    if (visible_is_left) {
      corridor.right = virt_result.boundary;
      corridor.right_ok = true;
    } else {
      corridor.left = virt_result.boundary;
      corridor.left_ok = true;
    }
  }
}
```

---

## 9. Stage 4: Centerline Build

> **파일**: `src/corridor/centerline_builder.cpp`, `include/track_planning/corridor/centerline_builder.hpp`

### Case 분기

| Case | 조건 | 방법 |
|------|------|------|
| **Case 1** | `both_ok && left >= 2 && right >= 2` | DTR 외심 (Delaunay Triangulation Racing) |
| **Case 2** | `virtual_used && visible >= 2` | 실측 경계에서 법선 오프셋 |
| **Case 3** | 그 외 | 실패 (빈 CenterlineResult) |

### Case 1: DTR 외심 기반 (`build_from_pair`)

> 참고: arXiv:2505.24320v1 (DTR 논문)

**6단계 알고리즘**:

```
build_from_pair(left, right, params)
│
├─ Step 1: 리샘플링
│   └─ left_rs, right_rs = resample_polyline(..., resample_ds)
│
├─ Step 1.5: 경계 교차 검사
│   └─ polylines_cross(left_rs, right_rs) → true면 조기 종료
│
├─ Step 2: Constrained Delaunay Triangulation (CDT)
│   ├─ 정점: [0..n_left-1] = 좌측, [n_left..n_total-1] = 우측
│   ├─ 제약 엣지: 좌측 연속, 우측 연속
│   ├─ 닫기 엣지: 시작↔시작, 끝↔끝 연결
│   └─ cdt.eraseOuterTrianglesAndHoles()
│
├─ Step 3: 삼각형 필터링
│   ├─ Filter A (혼합 클래스): 좌+우 꼭짓점 모두 포함해야 통과
│   └─ Filter B (기하 조건):
│       ├─ sides 정렬: s[0] <= s[1] <= s[2]
│       ├─ isosceles_like: s[2]/s[1] < 1.5
│       ├─ pointed: s[2]/s[0] > 2.0
│       ├─ large_area: area > 0.01 m²
│       └─ 통과 조건: (isosceles_like && pointed) || large_area
│
├─ Step 4: 외심(Circumcenter) 계산
│   └─ cc = circumcenter(a, b, c) → 세 꼭짓점에서 등거리인 점
│
├─ Step 5: 삼각형 인접 그래프 체이닝
│   ├─ 시작: ego(0,0)에서 외심이 가장 가까운 유효 삼각형
│   ├─ 양방향 인접 워크 (Triangle::neighbors[3])
│   └─ chain = reverse(path_b) + start + path_a
│
└─ Step 6: 리샘플링 + 3점 이동평균 스무딩
    └─ result.center = resample → smooth → valid 확인
```

**삼각형 필터링 직관**:

```
트랙을 가로지르는 삼각형:
    L ──────── L         좌측 경계
     \  ╲   /
      \   X  ← 외심 = 중앙선 근방
       \ / ╲
    R ──────── R         우측 경계

- 두 긴 변(s[1], s[2])은 비슷 → isosceles_like
- 가장 긴 변(s[2]) ≫ 가장 짧은 변(s[0]) → pointed
- 트랙을 가로지르는 얇은 삼각형의 외심이 중앙선에 가장 가까움
```

**인접 그래프 체이닝**: 삼각형 A와 삼각형 B가 엣지를 공유하면 인접 → 인접한 유효 삼각형의 외심을 연결 = Voronoi 엣지 = medial axis (중앙축)

### Case 2: 한쪽 오프셋 (`build_from_one_side`)

```cpp
const double sgn = visible_is_left ? -1.0 : 1.0;
const double offset = sgn * (w_hat / 2.0);

for (size_t i = 0; i < vis_rs.size(); ++i) {
  const Point2D n = rotate90(tangents[i]);            // 좌측 법선
  result.center.push_back(vis_rs[i] + offset * n);    // w_hat/2 오프셋
}
```

- 실측 경계에서 법선 방향으로 `w_hat / 2` 이동 → 중앙선 추정

---

## 10. Stage 5: Raw Path 생성

> **파일**: `src/nodes/local_planner_node.cpp` → `on_timer()` (5단계)

```cpp
std::vector<Point2D> raw_path;
if (centerline_result.valid) {
  raw_path = centerline_result.center;  // 센터라인을 직접 경로로 사용
}
```

- DTR 센터라인이 유효하면 그대로 raw_path로 전달
- 별도의 경로 최적화 없이 후처리 단계로 넘김

---

## 11. Stage 6: Postprocess

> **파일**: `src/postprocess/path_postprocessor.cpp`, `include/track_planning/postprocess/path_postprocessor.hpp`

### 4단계 파이프라인

```
process(raw_path, prune_max_dev, smooth_window, resample_ds)
│
├─ [단계 1] Prune (가지치기)
│   └─ Greedy shortcut pruning
│
├─ [단계 2] Smooth (이동평균 스무딩)
│   └─ Moving average (양 끝점 보존)
│
├─ [단계 3] Resample (균일 간격 재샘플링)
│   └─ resample_polyline(smoothed, resample_ds)
│
└─ [단계 4] Yaw 계산
    └─ polyline_tangents() → heading() → yaw[i]
```

### 11.1 Prune: Greedy Shortcut Pruning

**목적**: 수직 편차 허용 범위 내에서 최대한 직선으로 단순화

```
입력: A ── B ── C ── D ── E ── F

검사: A→F 직선에서 B,C,D,E의 수직 거리 모두 ≤ max_dev?
  YES → A─────────────────F  (B,C,D,E 삭제)
  NO  → A→E 시도, A→D 시도, ... (역순으로 탐색)
```

**알고리즘 (의사코드)**:

```
result = [pts[0]]
i = 0
while i < n-1:
  best_j = i+1
  for j = n-1 downto i+2:       // 가장 먼 점부터 시도
    dir = normalize(pts[j] - pts[i])
    for k = i+1 to j-1:          // 중간점 수직 거리 검사
      perp = |cross2(pts[k]-pts[i], dir)|
      if perp > max_dev: break
    if all_ok: best_j = j; break
  result.append(pts[best_j])
  i = best_j
```

**수직 거리 계산**:

```
perp = |dx * dir.y - dy * dir.x|    (2D 외적의 절댓값)
```

**파라미터**: `postprocess.prune_max_dev` (기본 0.15m)

### 11.2 Smooth: 이동평균

```cpp
for (int i = 1; i < n - 1; ++i) {  // 양 끝점 보존
  int lo = max(0, i - half);
  int hi = min(n-1, i + half);
  result[i] = mean(pts[lo..hi]);    // X, Y 각각 평균
}
```

- 양 끝점(start, goal)은 원본 유지
- 경계 근처는 비대칭 윈도우 (가용한 포인트만)
- **파라미터**: `postprocess.smooth_window` (기본 5)

### 11.3 Resample: 균일 간격 재샘플링

`geometry.hpp`의 `resample_polyline()` 사용 → Segment Walking 알고리즘으로 `resample_ds` 간격의 균일한 waypoint 생성

**파라미터**: `postprocess.resample_ds` (기본 0.10m)

### 11.4 Yaw 계산

```cpp
auto tangents = polyline_tangents(result.path);
for (size_t i = 0; i < tangents.size(); ++i) {
  result.yaw[i] = heading(tangents[i]);  // atan2(y, x)
}
```

- 전방 차분으로 접선 단위벡터 계산 → `atan2`로 heading 각도 변환

---

## 12. Stage 7: Safety Check

> **파일**: `include/track_planning/safety/safety_checker.hpp` (header-only)

### 검사 순서

```
check(path, params)
│
├─ [1] 경로 유효성: !valid || size < 2 → STOP
│
├─ [2] 곡률 실현 가능성:
│   ├─ κ_max = compute_max_curvature(path)  → Menger 곡률
│   ├─ κ_limit = 1 / r_min
│   └─ κ_max > κ_limit → INFEASIBLE
│
└─ [3] 목표 속도 계산:
    ├─ v_target = v_max
    ├─ v_curve = sqrt(a_lat_max / κ_max)  → 곡률 속도 제한
    ├─ v_target = min(v_target, v_curve)
    ├─ v_target = clamp(v_target, 0, v_max)
    └─ state = OK
```

### Menger 곡률

세 연속 포인트 A, B, C에서:

```
BA = B - A,  CB = C - B

κ = 2 * |cross(BA, CB)| / (|BA| * |CB| * |AC|)
```

- 원리: 외접원 반경 R의 역수
- 경로 전체에서 최대 κ를 구함

### 목표 속도 계산

```
v_curve = sqrt(a_lat_max / κ_max)

원심력 조건: a_lat = v² × κ ≤ a_lat_max
→ v ≤ sqrt(a_lat_max / κ)
```

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `speed.v_max` | 1.60 m/s | T870 최대 속도 |
| `speed.a_lat_max` | 2.0 m/s² | 최대 횡가속도 |
| `vehicle.r_min()` | `wheelbase / tan(delta_max)` | 최소 회전반경 (~2.68m) |

---

## 13. Stage 8: Publish

> **파일**: `src/nodes/local_planner_node.cpp` → `on_timer()` (8단계)

### 핵심 퍼블리시

```cpp
// 최종 경로
auto path_msg = std::make_unique<nav_msgs::msg::Path>(
  to_path_msg(pp_result.path, "base_link", stamp));
pub_path_->publish(std::move(path_msg));

// 플래너 상태
auto status_msg = std::make_unique<track_msgs::msg::PlannerStatus>();
status_msg->status = static_cast<uint8_t>(safety.state);
status_msg->reason = safety.reason;
pub_status_->publish(std::move(status_msg));
```

### 디버그 퍼블리시 (Lazy Publishing)

```cpp
// 구독자가 있을 때만 메시지 생성 (불필요한 메시지 생성 방지)
if (pub_dbg_corridor_left_->get_subscription_count() > 0) {
  pub_dbg_corridor_left_->publish(...);
}
```

- `get_subscription_count() > 0`: RViz2가 토픽을 구독 중일 때만 메시지 생성
- 디버그 토픽: corridor_left, corridor_right, centerline, virtual_used

---

## 14. 기하 유틸리티

> **파일**: `include/track_planning/common/geometry.hpp` (header-only)

### 스칼라/벡터 연산

| 함수 | 수식 | 용도 |
|------|------|------|
| `dot2(a, b)` | `ax*bx + ay*by` | 내적 (s 성분 계산) |
| `cross2(a, b)` | `ax*by - ay*bx` | 외적 (d 성분, 수직 거리) |
| `norm(v)` | `√(x² + y²)` | 벡터 크기 |
| `dist(a, b)` | `√(Δx² + Δy²)` | 두 점 거리 |
| `normalize(v)` | `v / |v|` | 단위벡터 |
| `rotate90(v)` | `(-y, x)` | CCW 90도 회전 (좌측 법선) |

### 각도 연산

| 함수 | 수식 | 범위 |
|------|------|------|
| `wrap_pi(θ)` | `fmod → [-π, π]` | 각도 정규화 |
| `angle_diff(a, b)` | `wrap_pi(b - a)` | 부호 있는 각도 차이 |
| `heading(v)` | `atan2(v.y, v.x)` | 벡터 → heading |

### 폴리라인 연산

| 함수 | 설명 |
|------|------|
| `polyline_length(pts)` | 총 호 길이 |
| `resample_polyline(pts, ds)` | Segment Walking으로 균일 간격 리샘플링 |
| `polyline_tangents(pts)` | 각 점의 접선 단위벡터 (전방 차분) |
| `regress_tangent(pts, n_reg)` | 마지막 N개 점에서 평균 접선 방향 회귀 |
| `circumcenter(a, b, c)` | 삼각형 외심 (DTR에서 사용) |
| `segments_intersect(...)` | 두 선분 교차 판정 (CCW 기반) |
| `polylines_cross(a, b)` | 두 폴리라인 교차 여부 |
| `compute_median_width(L, R)` | 좌/우 경계의 중앙값 폭 |

---

## 15. 파라미터 레퍼런스

> **파일**: `config/planning.yaml`, `include/track_planning/common/params.hpp`

### ROI

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `roi.x_min` | -1.0 m | 후방 ROI 한계 |
| `roi.x_max` | 10.0 m | 전방 ROI 한계 |
| `roi.y_min` | -4.0 m | 우측 ROI 한계 |
| `roi.y_max` | 4.0 m | 좌측 ROI 한계 |
| `roi.resolution` | 0.10 m/cell | 격자 해상도 |

### Vehicle (T870)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `vehicle.width` | 0.50 m | 차량 폭 |
| `vehicle.wheelbase` | 0.87 m | 축간 거리 |
| `vehicle.delta_max` | 0.314 rad (~18도) | 최대 조향각 |
| (계산값) `r_min()` | ~2.68 m | 최소 회전반경 = L / tan(δ_max) |

### Corridor Seed

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `corridor.seed.r_seed` | 5.0 m | 최대 탐색 반지름 |
| `corridor.seed.r_seed_step` | 1.0 m | 반경 확장 단계 |
| `corridor.seed.forward_range` | 1.047 rad (~60도) | 전방 탐색 각도 범위 |
| `corridor.seed.w_dist` | 1.0 | 거리 가중치 |
| `corridor.seed.w_center` | 1.0 | 중앙선 근접 가중치 |

### Corridor Filter

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `corridor.filter.s_min` | 0.05 m | 최소 전방 진행거리 |
| `corridor.filter.s_max` | 2.0 m | 최대 전방 탐색거리 |
| `corridor.filter.d_max` | 2.0 m | 최대 횡방향 거리 |
| `corridor.filter.r_search` | 2.5 m | 원형 탐색 반경 |

### Corridor Score

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `corridor.score.w_s` | 1.0 | 전방 진행 가중치 (+) |
| `corridor.score.w_d` | 0.8 | 횡방향 편차 패널티 (-) |
| `corridor.score.w_a` | 0.5 | 각도 편차 패널티 (-) |
| `corridor.score.w_p` | 0.3 | 예측 오차 패널티 (-) |
| `corridor.score.theta_max` | 1.047 rad | 각도 정규화 상한 |
| `corridor.score.step_pred` | 0.5 m | 예측 스텝 거리 |

### Virtual Boundary

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `virtual.default_track_width` | 1.5 m | 기본 트랙 폭 (대회 규격) |
| `virtual.min_corridor_width` | 0.6 m | 최소 코리도 폭 |

### Centerline (DTR)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `centerline.tri_isosceles_ratio` | 1.5 | 이등변 비율 상한 |
| `centerline.tri_pointed_ratio` | 2.0 | 가늘기 비율 하한 |
| `centerline.tri_min_area` | 0.01 m² | 최소 면적 |

### Speed

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `speed.v_max` | 1.60 m/s | T870 최대 속도 |
| `speed.a_lat_max` | 2.0 m/s² | 최대 횡가속도 |

### Postprocess

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `postprocess.resample_ds` | 0.10 m | 리샘플링 간격 |
| `postprocess.smooth_window` | 5 | 이동평균 윈도우 크기 |
| `postprocess.prune_max_dev` | 0.15 m | Pruning 최대 수직 편차 |

### Timeouts

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `timeouts.perception_ms` | 300 ms | Perception 타임아웃 |

---

## 16. 토픽 인터페이스

### 구독 (입력)

| 토픽 | 타입 | 소스 | QoS |
|------|------|------|-----|
| `/perception/lane_boundaries` | `track_msgs/LaneBoundaryArray` | 카메라 | Reliable, depth=1 |
| `/perception/cones` | `track_msgs/ConeArray` | LiDAR | Reliable, depth=1 |

### 퍼블리시 (출력 - 핵심)

| 토픽 | 타입 | 소비자 | 설명 |
|------|------|--------|------|
| `/planning/path` | `nav_msgs/Path` | 컨트롤러 | 후처리 완료된 최종 경로 |
| `/planning/status` | `track_msgs/PlannerStatus` | 컨트롤러 | 상태 코드 + 이유 문자열 |

### 퍼블리시 (출력 - 디버그, Lazy)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/planning/debug/corridor_left` | `nav_msgs/Path` | 좌측 코리더 경계 |
| `/planning/debug/corridor_right` | `nav_msgs/Path` | 우측 코리더 경계 |
| `/planning/debug/centerline` | `nav_msgs/Path` | 계산된 센터라인 |
| `/planning/debug/virtual_used` | `std_msgs/Bool` | 가상 경계 사용 여부 |

---

## 17. 빌드 구조

> **파일**: `CMakeLists.txt`

### 라이브러리 의존성

```
local_planner_component  (ComposableNode, .so)
├── track_planning_corridor  (SHARED library)
│   ├── corridor_builder.cpp
│   ├── virtual_boundary.cpp
│   └── centerline_builder.cpp
└── track_planning_postprocess  (SHARED library)
    └── path_postprocessor.cpp
```

### 빌드 명령

```bash
cd ~/ev_ws/planning
colcon build --packages-select track_planning
```

### 실행

```bash
source ~/ev_ws/planning/install/setup.bash
ros2 launch track_planning planning.launch.py
```

### ComposableNodeContainer 패턴

`planning.launch.py`에서 `ComposableNodeContainer`를 사용:

```python
ComposableNodeContainer(
    name='planning_container',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='track_planning',
            plugin='track_planning::LocalPlannerNode',
            name='local_planner_node',
            parameters=[params],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
)
```

- `component_container`: 여러 노드를 단일 프로세스에서 실행
- `use_intra_process_comms: True`: 같은 컨테이너 내 Zero-copy 통신 활성화
- `RCLCPP_COMPONENTS_REGISTER_NODE` 매크로로 팩토리 함수 등록 → 런타임 동적 로드
