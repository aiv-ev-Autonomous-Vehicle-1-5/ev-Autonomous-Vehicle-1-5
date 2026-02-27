# Chaining 모듈 설계명세서 v2
## Component → Backbone → Branch 기반 좌/우 차선 경계 체이닝

> 대상: 제5회 국제 대학생 EV 자율주행 경진대회
> 프레임워크: ROS 2 Humble / C++ ComposableNode
> 작성일: 2026-02-28
> 전제: DBSCAN 클러스터링(→ MakeBBox)까지 완성됨
> 선행 문서: chaining_spec.md (v1, 단순 directed chaining)

---

## [CHN2-00] 문서 목적과 범위

본 문서는 `/home/aiv/ev_ws` 기반 자율주행 시스템에서 LiDAR 클러스터링 결과(`ev_msgs::BBoxArray`)와 카메라 차선 인식 결과(`ev_msgs::LaneBoundaryArray`)를 입력으로 받아, **좌/우 차선 경계를 branch까지 포함해 chaining**하는 모듈을 설계한다.

v1(chaining_spec.md)은 greedy/beam 방식으로 단일 경로만 추출했다. 이 방식은 `|ㅓ` 형태(정적장애물로 인한 중간 돌출/분기)에서 다음 문제가 발생한다:
- backbone이 돌출 가지로 빠져 `|ㄱ` 형태가 된다.
- 또는 돌출을 무시하고 `||`만 뽑는다.
- branch 정보가 보존되지 않아 장애물 형상 반영이 불가하다.

본 문서는 **Component → Backbone → Branch** 3단계 절차를 도입하여:
1. 같은 side의 모든 클러스터를 하나의 component로 묶고 (돌출 포함)
2. component 내에서 주행에 쓸 backbone 1개를 greedy chaining으로 추출하고
3. 나머지 노드를 branch로 보존한다.

문체는 "-다"로 통일한다.

---

## [CHN2-01] 핵심 요구사항

1. 좌/우 side를 분리한 상태로 chaining 결과를 생성해야 한다.
2. 곡선 구간에서도 연결이 끊기지 않아야 한다.
3. `|ㅓ`처럼 **중간 돌출/분기**가 있어도 같은 side 그룹으로 유지되어야 한다.
4. backbone(주 경계선) 1개만 뽑아서 주행(Costmap → Planner)에 쓸 수 있어야 한다.
5. 동시에 branch(부 경계선)를 **모두 보존**해 디버그/장애물 형상 반영에 쓸 수 있어야 한다.
6. 좌/우 cross-connection(교차 연결)을 억제해야 한다.
7. 파라미터는 ROS 2 YAML 파일로 튜닝 가능해야 한다.
8. 기존 `planning_lc_ver` 파이프라인에 통합 가능해야 한다.

---

## [CHN2-02] 입력/출력 정의

### [CHN2-02A] 입력 토픽 (v1과 동일)

| 토픽 | 메시지 타입 | 프레임 | 설명 |
|------|-----------|--------|------|
| `/perception/bboxes` | `ev_msgs::msg::BBoxArray` | `base_link` | LiDAR DBSCAN → MakeBBox 출력 (라바콘) |
| `/perception/lane_boundaries` | `ev_msgs::msg::LaneBoundaryArray` | `base_link` | 카메라 차선 인식 출력 |

### [CHN2-02B] 좌표계 (v1과 동일)

```
base_link (차량 중심)
  x: 전방(+), 후방(-)
  y: 좌측(+), 우측(-)
  z: 상방(+), 하방(-)

ego 위치 = (0, 0)
```

### [CHN2-02C] 내부 통합 포인트 구조체 (v1 확장)

```cpp
enum class PointType : uint8_t { CONE, LANE };

struct ChainPoint {
    double x;              // [m] base_link 기준
    double y;              // [m] base_link 기준
    PointType type;        // CONE 또는 LANE
    float confidence;      // 원본 신뢰도 [0.0~1.0]
    int32_t label;         // 원본 cluster_id (콘: BBox.label, 차선: -1)
    double size_x;         // AABB X [m] (콘만 유효, 차선: 0)
    double size_y;         // AABB Y [m] (콘만 유효, 차선: 0)
};
```

변환 규칙 (v1과 동일):
- `BBox` → `ChainPoint{position.x + tf_x, position.y + tf_y, CONE, confidence, label, size_x, size_y}`
- `LaneBoundary.points[i]` → `ChainPoint{p.x, p.y, LANE, boundary.confidence, -1, 0, 0}`

### [CHN2-02D] 출력

#### 주행용 출력 (토픽 없음, 내부 함수 호출)

`CostmapGenerator::generate()`에 **component 전체**(backbone + branches)를 직접 전달한다.
별도 토픽을 거치지 않는다 (기존 `on_timer()` 내부 호출 방식 유지).

```cpp
// component = 해당 side의 모든 포인트 (backbone + branch 포함)
costmap_generator_.generate(
    result.left.component,     // L_set 전체
    result.right.component,    // R_set 전체
    params_);
```

이렇게 하면 `|ㅓ` 돌출 부분(branch)에도 cost가 깔려서 planner가 장애물을 회피한다.

#### 디버그 토픽 (`publish_debug: true` 시)

| 토픽 | 메시지 타입 | 프레임 | 설명 |
|------|-----------|--------|------|
| `/chaining/debug/left_backbone` | `nav_msgs::msg::Path` | `base_link` | 좌측 backbone 폴리라인 |
| `/chaining/debug/right_backbone` | `nav_msgs::msg::Path` | `base_link` | 우측 backbone 폴리라인 |
| `/chaining/debug/left_branches` | `visualization_msgs::msg::MarkerArray` | `base_link` | 좌측 branch들 시각화 |
| `/chaining/debug/right_branches` | `visualization_msgs::msg::MarkerArray` | `base_link` | 우측 branch들 시각화 |
| `/chaining/debug/seeds` | `visualization_msgs::msg::MarkerArray` | `base_link` | seed + goal 마커 |

backbone + branches를 함께 보면 component 전체를 확인할 수 있으므로, 별도 component 마커는 두지 않는다. backbone + branches = left/right component는 lc_planner_node에서 내부 함수 호출로 넘겨준다.(토픽을 발행해서 넘겨주지 않는다.)

### [CHN2-02E] 내부 출력 구조체

```cpp
struct BranchInfo {
    int parent_backbone_idx;                 // backbone 상의 분기점 인덱스
    std::vector<ChainPoint> points;          // branch 포인트 (parent→끝 순서)
    double score;                            // branch 품질 (누적 비용 역수 등)
};

enum class StopReason {
    NO_CANDIDATE,       // 전방에 후보 없음
    ALL_GATED,          // 후보는 있으나 모든 게이트 탈락
    CYCLE,              // 이미 방문한 노드로 돌아옴
    MAX_LEN             // max_chain_len 도달
};

struct SideResult {
    std::vector<ChainPoint> component;       // side 전체 리샘플 포인트 (backbone+branches 전 edge 보간, costmap 전달용)
    std::vector<ChainPoint> backbone;        // 주 경계선 (seed→goal, 리샘플 완료, 디버그 시각화용)
    std::vector<BranchInfo> branches;        // 분기선들 (디버그 시각화용)
    int seed_idx;                            // seed 포인트 인덱스
    int goal_idx;                            // greedy chain 마지막 노드 (자동 결정)
    StopReason stop_reason;
};

struct ChainResult {
    SideResult left;
    SideResult right;
    bool valid;                              // 최소 한쪽 backbone 생성 성공
};
```

기존 `planning_lc_ver`와의 호환을 위해 `ChainResult`에서 component를 꺼내는 헬퍼:

```cpp
// CostmapGenerator 호환용: component 전체를 ChainedPoint로 변환
std::vector<ChainedPoint> get_left_chain() const {
    // left.component를 ChainedPoint로 변환 (backbone + branches 포함)
}
std::vector<ChainedPoint> get_right_chain() const {
    // right.component를 ChainedPoint로 변환
}
```

---

## [CHN2-03] ROS 2 파라미터 (YAML)

```yaml
chaining:
  # --- Seed 선택 ---
  side_seed_y: 0.3        # [m] |y| < 이 값이면 seed 후보 제외 (중앙 잡음 억제)

  # --- kNN + 게이트 ---
  k: 8                    # centroid kNN 후보 수
  d_max: 1.5              # [m] neighbor 최대 거리
  forward_cone_deg: 120.0 # [deg] 전방 cone 전체 각도 (±60°)
  theta_max_deg: 45.0     # [deg] 진행 방향 연속성 최대 허용각
  lateral_gate: 0.75       # [m] 횡방향 오차 한계

  # --- 비용함수 가중치 ---
  alpha: 1.0              # 거리 비용
  beta: 1.2               # 방향 오차 비용
  gamma: 0.6              # 횡오차 비용
  delta: 0.2              # 크기 변화 비용 (콘 전용)

  # --- Backbone 추출 (greedy chaining) ---
  lambda_side: 0.5        # side preference 가중치
                           # backbone이 바깥쪽(좌:y+, 우:y-) 경계를 선호하도록 한다.
                           # 이 항이 없으면 |ㅓ 에서 돌출 가지가 backbone에 빨려 들어간다.
                           # goal은 greedy chain의 마지막 노드로 자동 결정된다.

  # --- Branch 추출 ---
  branch_mode: "backbone_and_branches"
                           # "off": backbone만 추출, branch 무시
                           # "backbone_only": backbone만 추출, component는 보존
                           # "backbone_and_branches": backbone + 모든 branch 추출
  max_branch_len: 40      # branch 최대 길이 (포인트 수), 폭주 방지

  # --- 종료/제한 ---
  max_chain_len: 100      # backbone 최대 길이

  # --- 콘 우선순위 ---
  cone_priority: true     # 동일 영역에 콘+차선 공존 시 차선 제거

  # --- 신뢰도 필터 ---
  min_confidence: 0.1     # BBox/LaneBoundary 최소 신뢰도 컷오프

  # --- 리샘플링 ---
  resample_ds: 0.1        # [m] component 전체 리샘플 간격 (backbone + branch 연결 + branch 내부)

  # --- 디버그 ---
  publish_debug: true     # 디버그 마커 발행 여부
  dump_diagnostics: false # 진단 정보 로그 덤프
```

### 파라미터 기본값 선정 근거

| 파라미터 | 값 | 근거 |
|---------|-----|------|
| `lambda_side` | 0.5 | 너무 크면 곡선에서 backbone이 바깥쪽으로 편향, 너무 작으면 돌출이 backbone에 침투. 0.5에서 시작해 Gazebo에서 조정한다. |
| `max_branch_len` | 40 | 정적장애물(PE 드럼 직경 500mm) 주변 돌출은 보통 3~10개 클러스터. 40이면 충분한 여유이다. |
| `branch_mode` | `backbone_and_branches` | 디버깅/튜닝 단계에서는 branch 정보가 필수. 실전에서 성능 이슈 시 `backbone_only`로 전환한다. |

---

## [CHN2-04] 알고리즘 설계 (5단계)

### [CHN2-04A] 0단계: 전처리

1. **신뢰도 필터**: `confidence < min_confidence`인 입력 제거
2. **입력 통합**: BBox + LaneBoundary → `std::vector<ChainPoint>`

ROI 필터는 DBSCAN 클러스터링 단계에서 이미 적용되어 있으므로 chaining에서는 수행하지 않는다.

### [CHN2-04B] 1단계: 좌/우 Seed 선택

v1과 동일하다:
- Left 후보: `y ≥ +side_seed_y` 이고 `x ≥ 0`
- Right 후보: `y ≤ -side_seed_y` 이고 `x ≥ 0`
- seed = 각 후보 중 `argmin sqrt(x² + y²)`
- `cone_priority` 적용: 콘이 있으면 콘을 우선 seed로 선택

### [CHN2-04C] 2단계: Undirected Graph 구성

이 단계에서는 **undirected 그래프만 사전 구성**한다. directed 이웃 탐색은 4단계 greedy chaining에서 동적으로 수행한다.

#### 게이트 정의

| # | 게이트 | 조건 | 사용처 |
|---|--------|------|--------|
| G1 | 거리 게이트 | `dist(i,j) ≤ d_max` | undirected + directed |
| G2 | 전방 cone 게이트 | `angle(v_i, u_ij) ≤ forward_cone_deg / 2` | directed만 |
| G3 | 횡오차 게이트 | `lateral_offset(i→j) ≤ lateral_gate` | undirected + directed |

- **undirected**: G1 + G3 (2개만) — 방향 무관, 거리와 횡오차만 검사
- **directed**: G1 + G2 + G3 (3개 전부) — 진행 방향 기준 전방 cone까지 검사

#### Undirected Graph 구성 절차

```
// 입력: points (전처리 완료된 ChainPoint 배열, N개)
// 출력: undirected_adj[N] (인접 리스트, 양방향)

undirected_adj = [[] for _ in range(N)]

for each node i in 0..N-1:
    // Step 1: i의 kNN 후보 k개를 거리 기준으로 뽑는다
    neighbors = kNN(points, i, k)    // k=8, 거리 순 정렬

    for each j in neighbors:
        if j == i: continue

        // Step 2: 거리 게이트 (G1)
        if dist(i, j) > d_max: continue

        // Step 3: 횡오차 게이트 (G3)
        //   횡오차 = i→j 벡터의 y 성분 (x축 수직 방향 편차)
        //   undirected이므로 v_i 불필요, 단순 y 차이 절대값 사용
        if |points[j].y - points[i].y| > lateral_gate: continue

        // Step 4: 양방향 edge 추가 (중복 체크)
        if j not in undirected_adj[i]:
            undirected_adj[i].push_back(j)
            undirected_adj[j].push_back(i)
```

**핵심**: undirected에서는 **전방 cone 게이트(G2)를 적용하지 않는다**.
→ 방향과 무관하게 "가까이 있고 횡방향 범위 내"이면 연결한다.
→ `|ㅓ` 돌출 노드가 backbone과 같은 component로 묶이려면 방향 제약 없이 연결되어야 하기 때문이다.

#### 왜 undirected와 directed를 분리하는가?

| | undirected (2단계) | directed (4단계) |
|---|---|---|
| **목적** | component 추출 (같은 side 노드 묶기) | backbone 추출 (주행 경로 탐색) |
| **게이트** | G1 + G3 (2개) | G1 + G2 + G3 (3개) |
| **구성 시점** | 사전 구성 | greedy chaining 중 동적 구성 |
| **방향성** | 양방향 | 단방향 (v_i 기준 전방만) |

- **undirected가 필요한 이유**: greedy chaining(directed)만 사용하면 `|ㅓ`의 돌출 부분이 seed→forward 방향에서 도달 불가능할 수 있다. undirected로 연결성을 확보해야 "같은 side에 속하는 모든 클러스터"를 묶을 수 있다.
- **directed를 사전 구성하지 않는 이유**: greedy chaining에서 `v_i = normalize(current - prev)`로 매 step 진행 방향이 갱신된다. 사전에 고정 v_i로 directed graph를 구성하면 곡선에서 부정확하다. 따라서 directed 이웃 탐색은 greedy chaining 중에 동적으로 수행한다 (4단계 참조).

#### 비용함수 w(i,j) (v1과 동일)

```
C_d   = dist(i,j) / d_max
C_a   = angle(v_i, u_ij) / radians(theta_max_deg)
C_lat = lateral_offset / lateral_gate
C_size = |size_j - size_i| / (size_i + eps)   (콘 전용, LANE은 0)

w = α·C_d + β·C_a + γ·C_lat + δ·C_size
```

비용함수는 directed(greedy chaining)에서만 사용된다. undirected 그래프 구성에는 비용 계산이 필요 없다 (게이트 통과 여부만 판단).

### [CHN2-04D] 3단계: Side Component 추출 (v2 핵심)

**목적**: `|ㅓ`의 돌출을 포함해 같은 side의 모든 클러스터를 하나의 그룹으로 묶는다.

#### BFS Component 추출 절차

```
// 입력: undirected_adj (2단계에서 구성), left_seed, right_seed
// 출력: L_set (좌측 component 노드 인덱스), R_set (우측 component 노드 인덱스)

// ── visited를 양쪽에서 공유한다 (cross-connection 방지) ──
global_visited = set()

// ── ego에 가까운 seed를 먼저 처리한다 ──
if dist_to_ego(left_seed) ≤ dist_to_ego(right_seed):
    first_seed = left_seed,  second_seed = right_seed
else:
    first_seed = right_seed, second_seed = left_seed

// ── 1차 BFS: first side ──
first_set = BFS(undirected_adj, first_seed, global_visited)
global_visited = global_visited ∪ first_set

// ── 2차 BFS: second side ──
second_set = BFS(undirected_adj, second_seed, global_visited)
global_visited = global_visited ∪ second_set

// ── L_set / R_set 할당 ──
if first_seed == left_seed:
    L_set = first_set,  R_set = second_set
else:
    L_set = second_set, R_set = first_set
```

#### BFS 함수 상세

```
function BFS(adj, seed, global_visited):
    if seed in global_visited:
        return {}                        // seed가 이미 다른 side에 소속

    queue = [seed]
    component = {seed}
    global_visited.add(seed)

    while queue is not empty:
        current = queue.pop_front()

        for each neighbor in adj[current]:
            if neighbor in global_visited:
                continue                 // 이미 방문 (같은 side 또는 반대 side)
            global_visited.add(neighbor)
            component.add(neighbor)
            queue.push_back(neighbor)

    return component
```

#### visited 공유 정책

- L_set을 먼저 구성하면, R_set BFS 시 L_set에 이미 포함된 노드는 `global_visited`에 있으므로 자동으로 건너뛴다.
- → **cross-connection 원천 차단**: 한쪽 side에 속한 노드는 반대 side로 절대 넘어가지 않는다.
- ego에 가까운 seed를 먼저 처리하면 중앙 부근 경계 노드의 할당 편향을 줄일 수 있다.

#### 예시: `|ㅓ` 형태

```
좌측 차선:  ①──②──③──④──⑤    (직선)
                    │
                    ⑥              (돌출: PE 드럼)
                    │
                    ⑦

undirected_adj 연결 (G1+G3 통과):
  ①↔② ↔③↔④↔⑤
          ↕
          ⑥↔⑦

L_set = BFS(left_seed=①) = {①,②,③,④,⑤,⑥,⑦}
→ 돌출 노드 ⑥,⑦도 같은 component에 포함된다.
```

### [CHN2-04E] 4단계: Backbone 추출 — Greedy Chaining (v2 핵심)

**목적**: L_set(또는 R_set) 안에서 주행에 쓸 backbone 경로 1개를 뽑는다.

#### Step 4-1: Side Preference 비용

기본 비용 `w`에 side preference 항을 더해 `w'`를 만든다.

```
Left side:
  C_side = max(0, y_i - y_j) / Y_norm
  → y가 감소(안쪽으로 들어오는) 방향이면 페널티

Right side:
  C_side = max(0, y_j - y_i) / Y_norm
  → y가 증가(안쪽으로 들어오는) 방향이면 페널티

Y_norm = lateral_gate  (정규화 기준)

w' = w + lambda_side * C_side
```

**이 항의 역할**:
- `|ㅓ`에서 backbone이 돌출 가지(y가 안쪽으로 향하는)로 빠지는 것을 억제한다.
- backbone은 바깥쪽 경계(left: y+, right: y-)를 선호하게 된다.
- `lambda_side = 0`이면 v1과 동일하게 동작한다.

#### Step 4-2: Greedy Chaining

seed에서 시작하여 매 step마다 w' 최소인 이웃을 선택한다.
goal을 미리 정하지 않는다 — chain의 마지막 노드가 자동으로 goal이 된다.

```
backbone = [seed]
visited = {seed}
current = seed
v = (1, 0)                          // 초기 진행 방향 (base_link 전방)

while len(backbone) < max_chain_len:
    // 1) current의 kNN 후보 중 component 내 노드만 대상
    candidates = kNN(current, k) ∩ component_ids

    // 2) 게이트 적용 (dist + cone + lat, 3개 전부)
    //    v_current = normalize(current - prev), seed이면 (1,0)
    gated = apply_gates(current, candidates, v)

    // 3) visited 제거
    gated = gated - visited

    if gated is empty:
        stop_reason = NO_CANDIDATE (or ALL_GATED)
        break

    // 4) w' 계산 후 최소 비용 선택
    next = argmin(w'(current, j))  for j in gated

    // 5) 이동
    v = normalize(next - current)   // 진행 방향 갱신
    visited.add(next)
    backbone.append(next)
    current = next

goal_idx = backbone.back()          // 마지막 노드 = goal
```

**Dijkstra 대비 장점**:
- goal 선택 로직(`goal_select`, `goal_eta`) 불필요
- goal 도달 불가 시 fallback 로직 불필요
- 구현이 단순하고, v_i를 동적으로 갱신하므로 곡선 추종이 정확하다

### [CHN2-04F] 5단계: Branch 추출 (v2 신규)

`branch_mode == "backbone_and_branches"` 일 때만 수행한다.

#### Step 5-1: 잔여 노드 집합

```
rest = component_ids - backbone_ids
```

backbone에 포함되지 않은 노드들이다. `|ㅓ`에서 돌출 부분이 여기에 해당한다.

#### Step 5-2: 각 잔여 노드의 backbone 연결점 찾기

```
for each node n in rest:
    parent = undirected BFS로 n에서 가장 가까운 backbone 노드
    attach[n] = parent
```

#### Step 5-3: Branch 세그먼트 구성

같은 parent(backbone 연결점)를 공유하는 노드들을 묶어 branch를 구성한다.

```
for each unique parent p:
    branch_nodes = {n in rest | attach[n] == p}
    // branch 내 노드를 p에서의 거리 순으로 정렬
    branch = sort_by_distance_from(p, branch_nodes)

    if len(branch) > max_branch_len:
        branch = branch[:max_branch_len]   // 폭주 방지

    branches.push_back({
        parent_backbone_idx = backbone 내 p의 인덱스,
        points = branch,
        score = 1.0 / sum_of_costs
    })
```

#### Step 5-4: Branch 정보 저장

각 branch는 `BranchInfo{parent_backbone_idx, points, score}` 형태로 `SideResult.branches`에 저장된다.

### [CHN2-04G] 6단계: Component 리샘플링

backbone과 branch 구조가 확정된 후, **component 전체의 모든 edge**를 `resample_ds`(0.1m) 간격으로 선형 보간한다.

#### 리샘플 대상 edge

```
backbone edges:       ①→② → ②→③ → ③→④ → ④→⑤
branch 연결 edges:    ③→⑥  (backbone 노드 → branch 첫 노드)
branch 내부 edges:    ⑥→⑦ → ⑦→⑧
```

#### 리샘플링 절차

```
resampled_component = []

// 1) backbone edges 리샘플
for i in 0..len(backbone)-1:
    resample_edge(backbone[i], backbone[i+1], resample_ds)
    → 보간 포인트를 resampled_component에 추가

// 2) 각 branch의 연결 edge + 내부 edges 리샘플
for each branch in branches:
    parent = backbone[branch.parent_backbone_idx]
    first = branch.points[0]
    resample_edge(parent, first, resample_ds)    // 연결 edge

    for i in 0..len(branch.points)-1:
        resample_edge(branch.points[i], branch.points[i+1], resample_ds)
```

#### 타입 태깅

- 콘-콘 구간: CONE 타입 유지
- 그 외: LANE 타입

#### 결과

`SideResult.component`에 리샘플된 전체 포인트가 저장된다.
이 데이터가 CostmapGenerator에 전달되므로, backbone-branch 연결 부분에도 균일한 cost가 깔린다.

---

## [CHN2-05] 좌/우 체인 생성 절차 (프레임 단위)

```
1. 입력 수신: last_bboxes_, last_lanes_ 캐시
2. 전처리: 신뢰도 필터 → ChainPoint 통합
3. Seed 선택: left (y>0), right (y<0)
4. Candidate Graph 구성: directed + undirected
5. Component 추출: L_set = BFS(undirected, left_seed)
                    R_set = BFS(undirected, right_seed)  // visited 공유
6. Backbone 추출: Greedy chaining(w', seed→전방) 각 side
                    goal = chain 마지막 노드 (자동 결정)
7. Branch 추출: rest 노드 → backbone 연결점으로 그룹핑
8. Component 리샘플링: backbone edges + branch 연결 edges + branch 내부 edges 전부
9. Costmap 전달: 리샘플된 component → CostmapGenerator
10. 디버그 발행: backbone → Path, branches → MarkerArray
```

### [CHN2-05A] 충돌 해소

L_set과 R_set이 겹치면(공유 노드):
- greedy chaining 시 누적 비용이 더 작은 side에 우선권을 준다.
- 다른 side에서는 공유 노드를 제거하고 component에서 탈락시킨다.
- 탈락된 노드 이후의 backbone이 끊기면 해당 지점에서 truncate한다.

실전에서는 `side_seed_y`와 `lateral_gate`가 적절하면 겹침이 거의 발생하지 않는다.

---

## [CHN2-06] 데이터 구조 설계 (C++)

### [CHN2-06A] 그래프 표현

```cpp
struct ChainingGraph {
    // undirected: 연결성 전용, component 추출용 (사전 구성)
    std::vector<std::vector<int>> undirected;          // [node] -> [neighbors]

    // directed edge는 사전 구성하지 않는다.
    // greedy chaining 중에 kNN + 3게이트 + w' 계산을 동적으로 수행한다.
};
```

### [CHN2-06B] 클래스 시그니처

```cpp
class DirectionChainer {
public:
    struct Params {
        // Seed
        double side_seed_y;
        // Graph
        int k;
        double d_max;
        double forward_cone_rad;
        double theta_max_rad;
        double lateral_gate;
        // Cost
        double alpha, beta, gamma, delta;
        // Backbone (greedy chaining)
        double lambda_side;
        // Branch
        std::string branch_mode;      // "off", "backbone_only", "backbone_and_branches"
        int max_branch_len;
        // Limits
        int max_chain_len;
        bool cone_priority;
        double min_confidence;
        double resample_ds;
        // Debug
        bool publish_debug;
        bool dump_diagnostics;
    };

    explicit DirectionChainer(const Params & params);

    ChainResult chain(const std::vector<ChainPoint> & points) const;

private:
    Params params_;

    // 0단계: 전처리
    std::vector<ChainPoint> preprocess(
        const std::vector<ChainPoint> & points) const;

    // 1단계: seed 선택
    int find_seed(
        const std::vector<ChainPoint> & points,
        bool is_left) const;

    // 2단계: 그래프 구성
    ChainingGraph build_graph(
        const std::vector<ChainPoint> & points) const;

    // 3단계: component 추출 (undirected BFS)
    std::vector<int> extract_component(
        const ChainingGraph & graph,
        int seed_idx,
        std::vector<bool> & visited) const;

    // 4단계: backbone 추출 (greedy chaining)
    std::vector<int> extract_backbone(
        const ChainingGraph & graph,
        const std::vector<ChainPoint> & points,
        const std::vector<int> & component_ids,
        int seed_idx,
        bool is_left) const;

    // 5단계: branch 추출
    std::vector<BranchInfo> extract_branches(
        const ChainingGraph & graph,
        const std::vector<ChainPoint> & points,
        const std::vector<int> & component_ids,
        const std::vector<int> & backbone_ids) const;

    // 리샘플링
    std::vector<ChainPoint> resample(
        const std::vector<ChainPoint> & chain) const;
};
```

---

## [CHN2-07] 파이프라인 통합

### 방법 A: LineChainer 교체 (권장)

```
기존:
  LCPlannerNode::on_timer()
    → parse_input() → vector<ChainedPoint>
    → LineChainer::chain()
    → ChainResult{left_chain, right_chain, valid}
    → CostmapGenerator::generate(left_chain, right_chain)
    → MagneticPlanner::plan()

변경:
  LCPlannerNode::on_timer()
    → parse_input() → vector<ChainPoint>          // 필드 추가
    → DirectionChainer::chain()
    → ChainResult{left{component, backbone, branches}, right{...}}
    → CostmapGenerator::generate(
        result.left.component,                     // component 전체 전달 (backbone+branches)
        result.right.component)                    // → 돌출/branch에도 cost 깔림
    → MagneticPlanner::plan()
    → (debug) publish backbone as Path, branches as MarkerArray
```

`CostmapGenerator`와 `MagneticPlanner`는 수정하지 않는다.
component의 `ChainPoint`를 기존 `ChainedPoint`로 변환하는 헬퍼만 추가한다.

### 수정이 필요한 파일

| 파일 | 변경 내용 | 규모 |
|------|----------|------|
| `types.hpp` | `ChainedPoint`에 confidence/label/size 추가, `BranchInfo`/`SideResult` 추가 | ~30줄 |
| `params.hpp` | `Chainer` 구조체를 v2 파라미터로 교체 | ~25줄 |
| `lc_planner_node.cpp` | `parse_input()`에서 새 필드 채우기, backbone 전달, branch 마커 발행 | ~30줄 |
| `line_chainer.hpp/cpp` | **통째로 교체** → `DirectionChainer` | 새 파일 |
| `planning_lc.yaml` | 파라미터 항목 교체 | ~30줄 |
| `CMakeLists.txt` | 소스 파일명 변경 | 1줄 |

---

## [CHN2-08] 파라미터 튜닝 가이드

### 대회 환경 기반 초기값

| 항목 | 대회 규격 | 파라미터 영향 |
|------|----------|-------------|
| PE 드럼 직경 | 500mm | 돌출 branch의 크기 기준. 드럼 1개 = 클러스터 1~3개 |
| PE 드럼 높이 | 840mm | BBox size_z로 드럼/콘 구분 가능 |
| 교통 콘 배치 | 최소 3개 연속 | `d_max` = 콘 간격 × 2 |
| 차선 폭 | 1.5m | `lateral_gate` = 1.5m |
| 정적장애물 | 콘 3개 사이 드럼 배치 | `|ㅓ` 형태 발생 원인 |

### 튜닝 시나리오별 조정

| 증상 | 조정 방법 |
|------|----------|
| `|ㅓ`에서 돌출이 backbone에 침투 | `lambda_side` ↑ |
| `lambda_side` 과다 → 곡선 backbone 편향 | `lambda_side` ↓ |
| 돌출이 component에 아예 안 묶임 | `d_max` ↑ 또는 `lateral_gate` ↑ |
| branch가 너무 길게 뻗음 | `max_branch_len` ↓ |
| backbone이 너무 짧게 끊김 | `theta_max_deg` ↑, `d_max` ↑ |
| 곡선에서 반대 side 점프 | `lateral_gate` ↓, `beta` ↑ |
| 체인이 자주 끊김 | `d_max` ↑, `forward_cone_deg` ↑ |

---

## [CHN2-09] 로깅/디버깅 요구사항

### 필수 로그

| 레벨 | 내용 |
|------|------|
| `info` | seed (idx, x, y), goal (idx, x, y), L_set/R_set 크기, backbone 길이, branch 수, stop_reason |
| `debug` | 각 step 후보 탈락 통계(dist/theta/lat gate별), backbone edge별 (dist, angle, lat, w, w', C_side), junction 노드 수(degree≥3) |

### `dump_diagnostics: true` 시 추가 출력

- backbone edge 전체를 CSV 형태로 RCLCPP_DEBUG 출력
- branch별 parent_id, 길이, score 출력

### RViz 시각화 (`publish_debug: true` 시)

| 마커 | 형태 | 색상 | 설명 |
|------|------|------|------|
| left_backbone | LINE_STRIP | 녹색 | 좌측 backbone 폴리라인 |
| right_backbone | LINE_STRIP | 빨간색 | 우측 backbone 폴리라인 |
| left_branches | LINE_LIST | 연녹색 (각 branch 다른 밝기) | 좌측 branch들 |
| right_branches | LINE_LIST | 연빨간색 | 우측 branch들 |
| seeds | SPHERE (큰) | 좌:녹/우:빨 | seed 위치 |
| goals | SPHERE (큰) | 좌:파랑/우:주황 | goal 위치 |
| junctions | CUBE (작은) | 노란색 | degree≥3인 분기점 |

backbone + branches를 동시에 켜면 component 전체를 확인할 수 있다.

---

## [CHN2-10] 검증 시나리오

### [CHN2-10A] 테스트 케이스

| # | 시나리오 | 입력 특성 | 합격 기준 |
|---|---------|----------|----------|
| 1 | 직선 좌/우 콘 | 균일 간격 1.5m | backbone 100% 연결, branch 0개 |
| 2 | 곡선 좌/우 콘 | R=10m 원호 | backbone 90%+ 연결, branch 0개 |
| 3 | 곡선 + 결측 | 2~3개 콘 누락 | backbone 끊김 없이 gap 점프 |
| 4 | **`\|ㅓ` 돌출** | 좌측 직선 + 중간에 3개 콘이 안쪽 돌출 | **component에 돌출 포함, backbone은 직선 유지, 돌출은 branch 1개로 분리** |
| 5 | **`\|ㅓ` + 곡선** | 곡선 구간에서 돌출 | backbone은 곡선 유지, branch 분리 |
| 6 | 중앙 노이즈 | y≈0에 랜덤 콘 | cross-connection 0건 |
| 7 | 좁아지는 차선 | 좌우 3m → 1.5m | side 분리 유지 |
| 8 | Gazebo 환경 | `carsa_gazebo` empty.launch | 실시간 10Hz |

### [CHN2-10B] 정량 합격 기준

- backbone x 단조증가 경향: backbone 포인트의 x가 전체적으로 증가 (역행 비율 < 10%)
- `|ㅓ` 테스트: branch 수 ≥ 1, parent_backbone_idx가 돌출 근처
- cross 연결: L_set ∩ R_set = ∅ (visited 공유로 보장)
- 처리 시간: 프레임당 < 5ms

---

## [CHN2-11] 구현 순서

```
Phase 1 - 구조체/뼈대 (1일)
  ├─ ChainPoint, ChainingGraph, SideResult, BranchInfo, ChainResult 정의
  ├─ DirectionChainer 클래스 뼈대 + Params
  ├─ YAML 파라미터 로딩
  └─ 기존 LineChainer 호출부에 DirectionChainer 연결

Phase 2 - 그래프 + Component (1일)
  ├─ 전처리 (신뢰도 필터)
  ├─ seed 선택
  ├─ brute-force kNN + 게이트 → directed + undirected 그래프
  └─ BFS component 추출 (L_set, R_set)
      → 여기서 Gazebo 테스트: component가 |ㅓ 돌출을 포함하는지 확인

Phase 3 - Backbone (1일)
  ├─ side preference 비용 C_side 계산
  ├─ greedy chaining (seed→전방, w' 최소 선택)
  ├─ goal = chain 마지막 노드 (자동)
  └─ 리샘플링
      → Gazebo 테스트: backbone이 |ㅓ에서 직선 유지하는지 확인

Phase 4 - Branch + 통합 (1일)
  ├─ 잔여 노드 → backbone 연결점 매핑
  ├─ branch 세그먼트 구성
  ├─ CostmapGenerator에 component 전체 전달 (backbone+branches)
  └─ 디버그: backbone → Path, branches → MarkerArray 발행

Phase 5 - 디버깅/튜닝 (1일~)
  ├─ RViz 디버그 마커 전체 발행
  ├─ diagnostics 로그
  ├─ Gazebo 시나리오별 테스트
  └─ lambda_side, d_max, lateral_gate 튜닝
```

Phase 2까지 완료하면 component 시각화로 `|ㅓ` 포함 여부를 바로 확인할 수 있다.
Phase 3의 greedy chaining은 v1 LineChainer와 구조가 유사하므로 빠르게 구현 가능하다.

---

## [CHN2-12] v1 → v2 변경점 요약

| 항목 | v1 (chaining_spec.md) | v2 (본 문서) |
|------|----------------------|-------------|
| 그래프 | directed만 | directed + undirected 2종 |
| 탐색 방식 | greedy/beam (seed→전방 순차) | BFS component → greedy backbone (w' 비용) |
| `\|ㅓ` 대응 | 불가 (backbone만 or 무시) | component로 전체 포함 → backbone/branch 분리 |
| side preference | 없음 | `lambda_side * C_side` 항 추가 |
| branch 보존 | 없음 | `BranchInfo` 구조로 모든 가지 보존 |
| 출력 구조 | `ChainResult{left_chain, right_chain}` | `ChainResult{SideResult{component, backbone, branches}}` |
| goal 개념 | 없음 (전방으로 계속 진행) | greedy chain 마지막 노드 = goal (자동 결정) |
| 경로 알고리즘 | beam+lookahead | greedy chaining (w' 최소비용) |
| 파라미터 추가 | - | `lambda_side`, `branch_mode`, `max_branch_len` |

---

## [CHN2-13] 체크리스트 (오해 방지)

- [ ] undirected 그래프는 **component 추출 전용**이다. backbone 추출에는 greedy chaining(w' 비용)을 사용한다.
- [ ] `lambda_side`가 0이면 v1과 동등하게 동작한다. 점진적 활성화가 가능하다.
- [ ] greedy chaining은 **component 내 노드만** 대상으로 한다. component 밖 노드는 탐색하지 않는다.
- [ ] greedy chaining 시 v_i를 **동적으로 갱신**한다 (prev→current 방향). 곡선 추종이 정확하다.
- [ ] goal은 greedy chain의 **마지막 노드**로 자동 결정된다. 별도 goal 선택 로직이 없다.
- [ ] branch 추출은 **undirected 그래프**로 backbone 연결점을 찾는다. directed만 쓰면 역방향 연결을 놓친다.
- [ ] CostmapGenerator에는 **component 전체**(backbone+branches)를 넘긴다. 돌출/branch에도 cost가 깔려야 planner가 회피한다.
- [ ] component의 출력 형태는 기존 `left_chain`/`right_chain`과 **호환**된다. CostmapGenerator 수정 불필요하다.
- [ ] 좌표계는 **base_link** 기준이다. velodyne→base_link 변환은 MakeBBox에서 완료되어 있다.
