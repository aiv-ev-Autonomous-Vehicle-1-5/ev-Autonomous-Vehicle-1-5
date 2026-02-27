# Chaining 모듈 설계명세서
## Direction-Constrained Directed Graph 기반 좌/우 차선 경계 체이닝

> 대상: 제5회 국제 대학생 EV 자율주행 경진대회
> 프레임워크: ROS 2 Humble / C++ ComposableNode
> 작성일: 2026-02-28
> 전제: DBSCAN 클러스터링(→ MakeBBox)까지 완성됨

---

## [CHN-00] 문서 목적과 범위

본 문서는 `/home/aiv/ev_ws` 기반 자율주행 시스템에서, LiDAR 클러스터링 결과(`ev_msgs::BBoxArray`)와 카메라 차선 인식 결과(`ev_msgs::LaneBoundaryArray`)를 입력으로 받아, **Direction-Constrained Directed Graph** 방식으로 좌/우 차선 경계를 안정적으로 분리·연결하는 **Chaining 모듈**을 설계한다.

본 모듈은 기존 `planning_lc_ver::LineChainer`(Flood Fill BFS)를 대체하며, 다음 한계를 해결한다:
- 방향 제약 없음 → 곡선에서 반대 side로 점프
- 고정 반경 탐색 → 클러스터 간격 변화에 취약
- forward_angle 파라미터 미사용 → 후방 연결 발생
- beam/lookahead 미구현 → 분기점에서 오선택

문체는 "-다"로 통일한다.

---

## [CHN-01] 핵심 목표(기능 요구사항)

1. 좌/우 side를 분리한 상태로 체인을 생성해야 한다.
2. 곡선 구간(대회 코스 곡률)에서도 끊김 없이 연속성이 유지되어야 한다.
3. 울퉁불퉁한 경계(라바콘 배치 오차, 차선 검출 떨림)에서도 같은 side로 유지되어야 한다.
4. 노이즈 클러스터(외란)로 인한 cross-connection(좌우 교차 연결)을 억제해야 한다.
5. 출력은 좌 체인 / 우 체인 각각이 **순서가 있는 점 배열**(가까운 점부터 먼 점까지)이어야 한다.
6. 콘(BBox)과 차선(LaneBoundary)을 통합 처리하되, 콘 우선순위를 유지해야 한다.
7. 파라미터는 ROS 2 YAML 파일로 외부에서 튜닝 가능해야 한다.
8. 기존 파이프라인(`planning_lc_ver`)에 ComposableNode로 통합 가능해야 한다.

---

## [CHN-02] 입력/출력 정의

### [CHN-02A] 입력 토픽

| 토픽 | 메시지 타입 | 프레임 | 설명 |
|------|-----------|--------|------|
| `/perception/bboxes` | `ev_msgs::msg::BBoxArray` | `base_link` | LiDAR DBSCAN → MakeBBox 출력 (라바콘) |
| `/perception/lane_boundaries` | `ev_msgs::msg::LaneBoundaryArray` | `base_link` | 카메라 차선 인식 출력 |

#### BBox 필드 (입력 단위)

```
geometry_msgs/Point position   # 클러스터 중심 (x, y, z) [m], base_link 기준
float32 size_x                 # AABB X 크기 [m]
float32 size_y                 # AABB Y 크기 [m]
float32 size_z                 # AABB Z 크기 (높이) [m]
float32 confidence             # sigmoid 정규화 신뢰도 (0.0~1.0)
int32 label                    # 클러스터 ID (DBSCAN 라벨, 0~N)
```

#### LaneBoundary 필드 (입력 단위)

```
std_msgs/Header header         # 타임스탬프 + frame_id (base_link)
geometry_msgs/Point[] points   # 순서 정렬된 경계점 배열 [m]
float32 confidence             # 검출 신뢰도 (0.0~1.0)
```

### [CHN-02B] 좌표계 정의

```
base_link (차량 중심)
  x: 전방(+), 후방(-)
  y: 좌측(+), 우측(-)
  z: 상방(+), 하방(-)

ego 위치 = (0, 0)
```

velodyne → base_link 변환은 MakeBBox 단계에서 이미 적용되어 있다.
Chaining 모듈은 base_link 기준 좌표만 다룬다.

### [CHN-02C] 내부 통합 포인트 구조체

두 입력을 하나의 내부 구조체로 통합한다:

```cpp
enum class PointType : uint8_t { CONE, LANE };

struct ChainPoint {
    double x;              // [m] base_link 기준
    double y;              // [m] base_link 기준
    PointType type;        // CONE 또는 LANE
    float confidence;      // 원본 신뢰도 [0.0~1.0]
    int32_t label;         // 원본 cluster_id (콘만 유효, 차선은 -1)
    double size_x;         // AABB X [m] (콘만 유효, 차선은 0)
    double size_y;         // AABB Y [m] (콘만 유효, 차선은 0)
};
```

변환 규칙:
- `BBox` → `ChainPoint{position.x, position.y, CONE, confidence, label, size_x, size_y}`
- `LaneBoundary.points[i]` → `ChainPoint{p.x, p.y, LANE, boundary.confidence, -1, 0, 0}`

### [CHN-02D] 출력 토픽

| 토픽 | 메시지 타입 | 프레임 | 설명 |
|------|-----------|--------|------|
| `/chaining/left_boundary` | `ev_msgs::msg::LaneBoundary` | `base_link` | 좌측 체인 결과 |
| `/chaining/right_boundary` | `ev_msgs::msg::LaneBoundary` | `base_link` | 우측 체인 결과 |
| `/chaining/boundaries` | `ev_msgs::msg::LaneBoundaryArray` | `base_link` | 좌+우 통합 배열 |
| `/chaining/debug/left_marker` | `visualization_msgs::msg::Marker` | `base_link` | RViz 좌측 시각화 |
| `/chaining/debug/right_marker` | `visualization_msgs::msg::Marker` | `base_link` | RViz 우측 시각화 |
| `/chaining/debug/candidates` | `visualization_msgs::msg::MarkerArray` | `base_link` | 후보 edge 시각화 |

출력 `LaneBoundary.points[]`는 ego(0,0)에서 가까운 점부터 먼 점까지 정렬된 순서이다.
출력 `LaneBoundary.confidence`는 체인 내 포인트 평균 신뢰도이다.

---

## [CHN-03] ROS 2 노드 설계

### [CHN-03A] 노드 구조

```
패키지명: chaining (또는 기존 planning_lc_ver 내 모듈 교체)
노드명:   ChainingNode
타입:     rclcpp::Node (ComposableNode 대응)
```

기존 `planning_lc_ver` 파이프라인에서 `LineChainer` 클래스만 교체하는 방식도 가능하다.
그 경우 노드를 새로 만들지 않고, `LineChainer::chain()` 내부 알고리즘만 교체한다.

### [CHN-03B] 파라미터 (YAML)

```yaml
chaining:
  # --- ROI 필터 ---
  roi_min_x: 0.0          # [m] 전방만 처리 (후방 클러스터 제외)
  roi_max_x: 20.0         # [m] 전방 최대 거리
  roi_min_y: -8.0         # [m] 우측 한계
  roi_max_y: 8.0          # [m] 좌측 한계

  # --- Seed 선택 ---
  side_seed_y: 0.3        # [m] |y| < 이 값이면 seed 후보에서 제외 (중앙 잡음 억제)
                           # 차선 폭 1.5m → 좌우 경계 ±0.75m, 여유 두고 0.3m

  # --- kNN 후보 생성 ---
  k: 8                    # centroid kNN 후보 수
  d_max: 3.0              # [m] neighbor 최대 거리
                           # 라바콘 간격: 대회 규정상 콘 3개 연속 배치 → ~1.5m 간격 추정
                           # d_max = 간격 × 2 = 3.0m (결측 1개 허용)

  # --- 방향 제약 ---
  forward_cone_deg: 120.0 # [deg] 전방 cone 전체 각도 (±60°)
  theta_max_deg: 45.0     # [deg] 진행 방향 연속성 최대 허용각

  # --- 횡방향 게이트 ---
  lateral_gate: 1.5       # [m] 예상 진행선 대비 횡방향 오차 한계
                           # 차선 폭 1.5m → 같은 side 내 최대 편차

  # --- Beam Search ---
  beam_width: 3           # branch 발생 시 beam 폭
  lookahead: 3            # beam 평가 시 전방 L스텝 누적 비용

  # --- 비용함수 가중치 ---
  alpha: 1.0              # 거리 비용 가중치
  beta: 1.2               # 방향 오차 비용 가중치 (side 유지에 중요 → alpha보다 높게)
  gamma: 0.6              # 횡오차 비용 가중치 (beta × 0.5)
  delta: 0.2              # 클러스터 크기 변화 비용 가중치

  # --- 종료 조건 ---
  max_chain_len: 100      # 체인 최대 길이 (포인트 수)

  # --- 콘 우선순위 ---
  cone_priority: true     # 동일 반경 내 콘+차선 공존 시 차선 제거

  # --- 신뢰도 필터 ---
  min_confidence: 0.3     # 이 값 미만인 BBox/LaneBoundary는 제외
                           # MakeBBox의 sigmoid(n_mid=15)에서 ~8포인트 이하 제외

  # --- 리샘플링 ---
  resample_ds: 0.1        # [m] 출력 체인 리샘플 간격 (기존 LineChainer와 동일)

  # --- 디버그 ---
  log_level: "info"       # debug | info | warn | error
  publish_debug: true     # 디버그 마커 발행 여부
```

### [CHN-03C] 파라미터 기본값 선정 근거

| 파라미터 | 값 | 근거 |
|---------|-----|------|
| `roi_max_x` | 20.0m | VLP-16 유효 범위, DBSCAN ROI(-15~15)보다 약간 넓게 |
| `side_seed_y` | 0.3m | 차선 폭 1.5m의 1/5, 중앙 노이즈 제거 |
| `d_max` | 3.0m | 라바콘 평균 간격 ~1.5m × 2 (1개 결측 허용) |
| `forward_cone_deg` | 120° | 곡선 대응을 위해 넓게, 단 후방 60°는 차단 |
| `theta_max_deg` | 45° | 급커브(R=5m) 기준 연속 콘 간 방향 변화 최대치 |
| `lateral_gate` | 1.5m | 차선 폭과 동일 → 반대 side 점프 차단 |
| `beta` | 1.2 | alpha보다 높게 → 방향 연속성 우선 |
| `min_confidence` | 0.3 | sigmoid(k=0.3, n_mid=15)에서 ~8포인트 기준 |

---

## [CHN-04] 알고리즘 설계

### [CHN-04A] 전처리

#### Step 1: 신뢰도 필터
`confidence < min_confidence`인 입력은 제거한다.

#### Step 2: ROI 필터
centroid(x, y)가 ROI 밖이면 제거한다.
- 조건: `roi_min_x ≤ x ≤ roi_max_x`, `roi_min_y ≤ y ≤ roi_max_y`
- 주의: `roi_min_x = 0`이면 전방만 처리한다. 후방 클러스터는 체이닝 대상이 아니다.

#### Step 3: 입력 통합
BBox와 LaneBoundary를 `ChainPoint` 배열로 통합한다.

### [CHN-04B] Side 분리용 Seed 선택

좌/우 seed 후보군을 다음 규칙으로 만든다:
- Left 후보: `y ≥ +side_seed_y`
- Right 후보: `y ≤ -side_seed_y`
- 추가 조건: `x ≥ 0` (전방만)

각 side에서 seed는 ego(0,0) 기준 거리가 최소인 점으로 선택한다:
- `seed = argmin sqrt(x² + y²)`

콘이 있으면 콘을 우선 seed로 선택한다 (`cone_priority` 적용).

기존 LineChainer와 동일한 seed 전략이나, `side_seed_y`로 중앙 잡음을 억제하는 점이 개선이다.

### [CHN-04C] Directed kNN 그래프 구성

각 노드 i에서 후보 neighbor 집합 N(i)를 구성한다.

1. **kNN 검색**: centroid 기반으로 가까운 k개를 찾는다.
   - 구현: brute-force (포인트 수가 프레임당 ~50~200개이므로 KD-tree 불필요)
   - 기존 LineChainer도 brute-force 방식이었다.

2. **거리 게이트**: `dist(i,j) ≤ d_max`인 것만 유지한다.

3. **전방 cone 게이트**: 현재 진행 방향 기준으로 앞쪽 후보만 유지한다.

#### 진행 방향 벡터 v_i 정의

체인 생성 중 현재 노드가 i일 때:
- **(1)** 체인에 이전 노드 prev가 존재하면: `v_i = normalize(centroid_i - centroid_prev)`
- **(2)** prev가 없으면(시작 seed): `v_i = (1, 0)` (전방 단위벡터, base_link 기준 x축 양방향)

#### 전방 cone 조건

```
u_ij = normalize(centroid_j - centroid_i)
angle = acos(clamp(v_i · u_ij, -1, 1))
통과 조건: angle ≤ forward_cone_deg / 2
```

기존 LineChainer에서 `forward_angle` 파라미터가 정의만 되고 미사용이었던 문제를 해결한다.

### [CHN-04D] Edge 비용함수

후보 edge i→j의 비용 w(i,j):

```
C_d   = dist(i,j) / d_max                              # 거리 비용 [0~1]
C_a   = angle(v_i, u_ij) / radians(theta_max_deg)      # 방향 오차 [0~1+]
C_lat = lateral_offset / lateral_gate                   # 횡오차 [0~1]
C_size = |size_j - size_i| / (size_i + eps)             # 크기 변화 [0~∞] (콘만)

w = α·C_d + β·C_a + γ·C_lat + δ·C_size
```

#### 횡오차 C_lat 계산 ("side 유지"에 핵심)

```
r = centroid_j - centroid_i          # 상대 벡터
lateral_offset = |r - (r·v_i)·v_i|  # v_i 직교 성분의 크기
게이트: lateral_offset ≤ lateral_gate 아니면 후보 제거
비용:  C_lat = lateral_offset / lateral_gate
```

이 항이 같은 side 유지에 핵심적이다. 반대 side로 점프하려면 횡오차가 크므로 비용이 급증한다.

#### 크기 변화 C_size (콘 전용)

라바콘은 규격이 일정하다(직경 500mm, 높이 840mm). 크기가 급변하는 연결은 오연결 가능성이 높다.
- `size_i = sqrt(bbox.size_x² + bbox.size_y²)` (AABB 대각선)
- 차선 포인트(type == LANE)는 C_size = 0으로 처리한다.

### [CHN-04E] 다음 노드 선택 (Beam + Lookahead)

1. 후보들 중 w가 작은 순으로 정렬한다.
2. 상위 `beam_width`개를 beam으로 유지한다.
3. 각 beam 후보에 대해 `lookahead` 스텝까지 greedy로 이어붙인 누적 비용을 평가한다.
4. 누적 비용이 최소인 후보를 채택한다.

#### Lookahead 평가

```
for each candidate j in beam:
    score = w(i, j)
    current = j
    for step = 1 to lookahead:
        next = greedy_best_neighbor(current)  # beam 없이 min-w 선택
        if next == null: break
        score += w(current, next)
        current = next
    scores[j] = score

best = argmin scores
```

기존 LineChainer는 단순 반경 내 모든 이웃을 BFS로 추가했으나,
이 방식은 분기점에서 최적 경로를 선택한다.

### [CHN-04F] 종료 조건

체인 확장은 아래 조건 중 하나면 종료한다:
- 후보가 0개이다.
- 모든 후보가 게이트(theta/lat/dist)에서 탈락했다.
- 이미 방문한 포인트로 되돌아가려 한다(사이클 방지, `visited` set 사용).
- 체인 길이가 `max_chain_len`에 도달했다.
- x가 `roi_max_x`를 초과했다.

### [CHN-04G] 콘 우선순위 규칙

기존 LineChainer의 cone priority 로직을 유지하되 명확화한다:

```
if (cone_priority && 후보 목록에 CONE과 LANE이 공존):
    LANE 후보를 제거하고 CONE만 유지
```

이유: 콘은 물리적 장애물이므로 차선보다 경계 위치가 정확하다.

---

## [CHN-05] 좌/우 체인 생성 절차

### 프레임 단위 실행 (10Hz 타이머 콜백 내)

```
1. 입력 수신: last_bboxes_, last_lanes_ 캐시에서 가져오기
2. 전처리: 신뢰도 필터 → ROI 필터 → ChainPoint 통합
3. Left seed 선택 (y > 0, ego 최근접)
4. Right seed 선택 (y < 0, ego 최근접)
5. visited set 초기화 (좌/우 공유 → 중복 소속 방지)
6. Left seed에서 chaining 수행 → left_chain
7. Right seed에서 chaining 수행 → right_chain
   (left에서 사용한 포인트는 visited에 있으므로 right에서 재사용 불가)
8. 리샘플링 (resample_ds 간격)
9. 출력 발행
```

### [CHN-05A] 좌/우 visited 공유 정책

기존 LineChainer와 동일하게, `visited[]` 배열을 좌/우 체인이 공유한다.
- left 체인을 먼저 생성한다.
- left에서 사용된 포인트는 right에서 선택 불가하다.
- 이 방식은 cross-connection을 원천 차단한다.

단, left를 항상 먼저 하면 left에 유리한 편향이 생길 수 있다.
대안: 두 seed의 ego 거리를 비교해, 가까운 쪽을 먼저 체이닝한다.

### [CHN-05B] 리샘플링

기존 LineChainer의 리샘플링 로직을 유지한다:
- 체인 포인트 간 `resample_ds`(0.1m) 간격으로 선형 보간
- 콘-콘 구간은 CONE 타입, 그 외는 LANE 타입

```cpp
PointType seg_type = (a.type == CONE && b.type == CONE) ? CONE : LANE;
```

---

## [CHN-06] 데이터 구조 설계 (C++)

### [CHN-06A] 핵심 구조체

```cpp
// chaining/types.hpp

enum class PointType : uint8_t { CONE, LANE };

struct ChainPoint {
    double x, y;
    PointType type;
    float confidence;
    int32_t label;       // BBox의 cluster_id, LANE이면 -1
    double size_x, size_y;

    double dist_sq_to(double ox, double oy) const {
        return (x - ox) * (x - ox) + (y - oy) * (y - oy);
    }
};

struct EdgeCandidate {
    int from_idx;        // ChainPoint 배열 내 인덱스
    int to_idx;
    double dist;         // 유클리드 거리
    double angle;        // 진행 방향 대비 각도 [rad]
    double lateral;      // 횡오차 [m]
    double size_diff;    // 크기 변화율
    double cost;         // 최종 가중합 비용
};

enum class StopReason {
    NO_CANDIDATE,        // 후보 없음
    ALL_GATED,           // 모든 후보 게이트 탈락
    CYCLE,               // 사이클 감지
    MAX_LEN,             // 최대 길이 도달
    OUT_OF_ROI           // ROI 초과
};

struct ChainResult {
    std::vector<ChainPoint> left_chain;
    std::vector<ChainPoint> right_chain;
    StopReason left_stop;
    StopReason right_stop;
    bool valid;          // 최소 한쪽이라도 체인 생성 성공 여부
};
```

### [CHN-06B] 인덱싱/검색

포인트 수가 적다(프레임당 ~50~200개 ChainPoint):
- **brute-force kNN**으로 충분하다. KD-tree는 오버헤드만 증가한다.
- `visited`는 `std::vector<bool>` (인덱스 기반)로 관리한다.

### [CHN-06C] 클래스 시그니처

```cpp
// chaining/direction_chainer.hpp

class DirectionChainer {
public:
    struct Params {
        double roi_min_x, roi_max_x, roi_min_y, roi_max_y;
        double side_seed_y;
        int k;
        double d_max;
        double forward_cone_rad;   // forward_cone_deg를 radian으로 변환
        double theta_max_rad;      // theta_max_deg를 radian으로 변환
        double lateral_gate;
        int beam_width;
        int lookahead;
        double alpha, beta, gamma, delta;
        int max_chain_len;
        bool cone_priority;
        double min_confidence;
        double resample_ds;
    };

    explicit DirectionChainer(const Params & params);

    /// 프레임 단위 체이닝 실행
    ChainResult chain(const std::vector<ChainPoint> & points) const;

private:
    Params params_;

    /// 한쪽 side 체이닝
    std::vector<ChainPoint> chain_one_side(
        const std::vector<ChainPoint> & points,
        int seed_idx,
        std::vector<bool> & visited) const;

    /// kNN 후보 생성 + 게이트 적용
    std::vector<EdgeCandidate> find_candidates(
        const std::vector<ChainPoint> & points,
        int current_idx,
        const Eigen::Vector2d & direction,
        const std::vector<bool> & visited) const;

    /// beam+lookahead로 최적 후보 선택
    int select_best(
        const std::vector<ChainPoint> & points,
        int current_idx,
        const std::vector<EdgeCandidate> & candidates,
        const std::vector<bool> & visited) const;

    /// 리샘플링
    std::vector<ChainPoint> resample(
        const std::vector<ChainPoint> & chain) const;
};
```

---

## [CHN-07] 파이프라인 통합 방법

### 방법 A: 기존 LineChainer 교체 (권장)

`planning_lc_ver` 내 `LineChainer` 클래스를 `DirectionChainer`로 교체한다.

```
기존:
  LCPlannerNode::on_timer()
    → LineChainer::chain()      ← 이 부분만 교체
    → CostmapGenerator::generate()
    → MagneticPlanner::plan()

변경:
  LCPlannerNode::on_timer()
    → DirectionChainer::chain() ← 새 알고리즘
    → CostmapGenerator::generate()
    → MagneticPlanner::plan()
```

인터페이스 호환:
- 입력: `std::vector<ChainedPoint>` → `std::vector<ChainPoint>` (필드 추가)
- 출력: `ChainResult` (기존과 동일한 `left_chain`, `right_chain`, `valid` 구조)

### 방법 B: 독립 노드

별도 `chaining` 패키지로 만들어 `/perception/bboxes`를 구독하고 `/chaining/boundaries`를 발행한다.
이 경우 `planning_lc_ver`는 `/chaining/boundaries`를 구독하도록 수정한다.

**권장: 방법 A** (수정 범위가 작고 latency가 없다)

---

## [CHN-08] 파라미터 튜닝 가이드

### 대회 환경 기반 초기값

| 항목 | 대회 규격 | 파라미터 영향 |
|------|----------|-------------|
| 라바콘 직경 | 500mm | `delta` 비용 기준: 정상 콘의 AABB ≈ 0.5×0.5m |
| 라바콘 높이 | 840mm | BBox size_z ≈ 0.84m (크기 검증용) |
| 차선 폭 | 1.5m | `lateral_gate` = 1.5m, `side_seed_y` = 0.3m |
| 콘 배치 | 최소 3개 연속 | `d_max` = 콘 간격 × 2 |
| 차선 표시 | 10cm 흰색 테이프 | camera 인식 결과의 confidence 기준 |

### 튜닝 시나리오별 조정

| 증상 | 조정 방법 |
|------|----------|
| 곡선에서 반대 side로 점프 | `lateral_gate` ↓, `beta` ↑, `gamma` ↑, `forward_cone_deg` ↓ |
| 체인이 자주 끊김 | `d_max` ↑, `theta_max_deg` ↑, `lookahead` ↑ |
| 돌출 점이 다른 그룹으로 분리 | `lateral_gate` 약간 ↑, `gamma` ↓ |
| seed가 중앙 잡음에 잡힘 | `side_seed_y` ↑ |
| 노이즈 클러스터가 체인에 포함 | `min_confidence` ↑, `d_max` ↓ |

---

## [CHN-09] 로깅/디버깅 요구사항

### 필수 로그 (log_level에 따라)

| 레벨 | 내용 |
|------|------|
| `info` | seed 선택 결과 (id, 좌표), 체인 길이, 종료 사유 |
| `debug` | 각 step의 후보 수, 탈락 사유 통계(dist/theta/lat gate별), 선택 edge의 (dist, angle, lat, cost) |

### RViz 시각화 (`publish_debug: true` 시)

| 마커 | 설명 |
|------|------|
| `left_marker` | LINE_STRIP, 녹색, 좌측 체인 폴리라인 |
| `right_marker` | LINE_STRIP, 빨간색, 우측 체인 폴리라인 |
| `candidates` | POINTS, 노란색, 현재 step 후보 포인트 (마지막 프레임) |
| seed 마커 | SPHERE, 크게, 좌(녹)/우(빨) seed 위치 |

---

## [CHN-10] 검증 시나리오

### [CHN-10A] 테스트 케이스

| # | 시나리오 | 입력 특성 | 합격 기준 |
|---|---------|----------|----------|
| 1 | 직선 좌/우 콘 | 균일 간격 1.5m, 좌우 3m 폭 | 양쪽 체인 100% 연결 |
| 2 | 곡선 좌/우 콘 | R=10m 원호, 콘 10개씩 | 양쪽 체인 90% 이상 연결 |
| 3 | 곡선 + 결측 | 2~3개 콘 누락 | 끊김 없이 점프 |
| 4 | 돌출 경계 | 일부 콘이 0.3m 바깥으로 튀어나옴 | 같은 side 유지 |
| 5 | 중앙 노이즈 | y≈0 위치에 랜덤 콘 3개 | cross-connection 0건 |
| 6 | 좁아지는 차선 | 좌우 간격 3m → 1.5m | side 분리 유지 |
| 7 | Gazebo 환경 | `carsa_gazebo` empty.launch | 실시간 10Hz 동작 |

### [CHN-10B] 정량 합격 기준

- 체인 길이: 기대 길이 대비 90% 이상
- side 유지: left_chain의 y는 90% 이상 양수, right_chain의 y는 90% 이상 음수
- cross 연결: 좌/우 체인 간 공유 포인트 = 0
- 처리 시간: 프레임당 < 5ms (10Hz 여유 확보)

---

## [CHN-11] 구현 순서

```
Phase 1 - 뼈대 (1일)
  ├─ ChainPoint, EdgeCandidate, ChainResult 구조체 정의
  ├─ DirectionChainer 클래스 뼈대 + Params 구조체
  ├─ YAML 파라미터 로딩
  └─ 기존 LineChainer 호출부에 DirectionChainer 연결

Phase 2 - 핵심 알고리즘 (1~2일)
  ├─ ROI 필터 + seed 선택
  ├─ brute-force kNN 후보 생성
  ├─ 진행 방향 v_i + forward cone 게이트
  ├─ lateral 게이트 + 비용함수
  └─ greedy chaining (beam 없이 min-w 선택)

Phase 3 - 안정화 (1일)
  ├─ beam + lookahead 추가
  ├─ 콘 우선순위 로직
  ├─ 리샘플링
  └─ visited 공유 정책

Phase 4 - 디버깅/튜닝 (1일~)
  ├─ RViz 디버그 마커 발행
  ├─ 로그 출력
  ├─ Gazebo 테스트
  └─ 파라미터 튜닝
```

Phase 2의 greedy 버전이 동작하면 즉시 Gazebo에서 테스트한다.
beam+lookahead는 greedy가 불안정한 구간이 확인된 후 추가해도 된다.

---

## [CHN-12] 체크리스트 (오해 방지)

- [ ] 진행 방향 v_i는 **체인 진행 방향(prev→current 벡터)**이 1순위다. PCA 방향이 아니다.
- [ ] cone 게이트와 lateral 게이트는 **둘 다 필수**다. 하나라도 빠지면 곡선에서 점프 증가한다.
- [ ] 비용 각 항은 **0~1 범위로 정규화**해야 가중치 튜닝이 안정적이다.
- [ ] 좌/우를 독립으로 만들되, **visited set 공유**로 cross-connection을 차단한다.
- [ ] 좌표계는 **base_link** 기준이다. velodyne→base_link 변환은 이미 MakeBBox에서 처리되어 있다.
- [ ] 기존 `planning_lc_ver`의 `ChainResult` 구조와 호환되어야 한다.
- [ ] Gazebo(`carsa_gazebo`)에서 먼저 검증 후 실차 테스트로 넘어간다.

---

## [CHN-13] 기존 LineChainer 대비 변경점 요약

| 항목 | 기존 LineChainer | DirectionChainer |
|------|-----------------|------------------|
| 탐색 방식 | Flood Fill BFS (모든 이웃 추가) | Directed Graph (최적 1개 선택) |
| 방향 제약 | 없음 (forward_angle 미사용) | forward cone + theta_max 적용 |
| 횡방향 제약 | 없음 | lateral_gate로 side 점프 차단 |
| 후보 선택 | 반경 내 전부 추가 | beam + lookahead 비용 최소화 |
| 비용함수 | 없음 (거리만) | α·거리 + β·방향 + γ·횡오차 + δ·크기 |
| seed 보호 | y==0 제외만 | side_seed_y로 중앙 대역 제거 |
| 신뢰도 필터 | 없음 | min_confidence 컷오프 |
| 디버그 | debug/left_chain, right_chain | + seed 마커, 후보 마커, edge 로그 |
