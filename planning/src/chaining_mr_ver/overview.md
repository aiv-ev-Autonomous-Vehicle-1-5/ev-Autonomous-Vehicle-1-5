# chaining_mr_ver 패키지 구현 계획

## Context

기존 `planning_mr_ver`는 모든 cone/lane 점을 개별 자석으로 costmap에 올리는데, 라바콘 간격이 듬성듬성하면 Gaussian 감쇠(3σ≈3m) 사이에 cost=0인 "자기장 틈"이 생겨 경로가 바깥으로 이탈한다. 이를 해결하기 위해 costmap 생성 전에 **LineChainer** 단계를 추가하여 좌/우 경계점을 독립적으로 nearest-neighbor 체이닝한 뒤 0.1m 간격으로 리샘플링하여 연속적인 자기장 벽을 형성한다.

---

## 파이프라인 비교

```
[기존 planning_mr_ver]
  Stale → Parse(all) → Costmap(개별점) → Planner → Postprocess → Safety → Publish

[신규 chaining_mr_ver]
  Stale → Parse(L/R분리) → LineChainer → Resample → Costmap(체인) → Planner → Postprocess → Safety → Publish
```

---

## 파일 구조

```
planning/src/chaining_mr_ver/
├── CMakeLists.txt
├── package.xml
├── config/planning_lc.yaml
├── launch/planning_lc.launch.py
├── include/chaining_mr_ver/
│   ├── common/
│   │   ├── types.hpp              ← 확장: +PointType, +ChainedPoint, +ChainResult
│   │   ├── params.hpp             ← 확장: +Chainer 섹션
│   │   ├── geometry.hpp           ← 확장: +regress_direction()
│   │   └── debug_publish.hpp      ← 복사 (namespace 변경만)
│   ├── chainer/
│   │   └── line_chainer.hpp       ← 완전 신규
│   ├── costmap/
│   │   └── costmap_generator.hpp  ← 인터페이스 변경
│   ├── planner/
│   │   └── magnetic_planner.hpp   ← 복사 (namespace 변경만)
│   ├── postprocess/
│   │   └── path_postprocessor.hpp ← 복사 (namespace 변경만)
│   ├── safety/
│   │   └── safety_checker.hpp     ← 복사 (namespace 변경만)
│   └── nodes/
│       └── lc_planner_node.hpp    ← 파이프라인 재구성
└── src/
    ├── chainer/line_chainer.cpp   ← 완전 신규
    ├── costmap/costmap_generator.cpp ← 수정
    ├── planner/magnetic_planner.cpp  ← 복사 (namespace 변경만)
    ├── postprocess/path_postprocessor.cpp ← 복사 (namespace 변경만)
    └── nodes/lc_planner_node.cpp     ← 파이프라인 재구성
```

---

## 구현 순서 (Phase별)

### Phase 1: 기반 타입 + 유틸리티 (4파일)

**1-1. types.hpp** — 기존 planning_mr_ver 복사 + 아래 추가:
```cpp
enum class PointType : uint8_t { CONE = 0, LANE = 1 };

struct ChainedPoint {
  double x = 0.0, y = 0.0;
  PointType type = PointType::LANE;
  Point2D to_point2d() const { return {x, y}; }
};

struct ChainResult {
  std::vector<ChainedPoint> left_chain, right_chain;
  bool valid = false;
};
```

**1-2. params.hpp** — 기존 복사 + Chainer 섹션 추가:
```cpp
struct Chainer {
  double search_radius = 1.0;      // [m] 초기 탐색 반경
  double search_radius_step = 0.5;  // [m] 반경 증가 스텝
  double search_radius_max = 5.0;   // [m] 최대 탐색 반경
  double forward_angle = M_PI/2;    // [rad] 전방 각도 (90°)
  int min_regress_pts = 2;
  int max_regress_pts = 7;
  double resample_ds = 0.1;        // [m] 체인 리샘플 간격
} chainer;
```

**1-3. geometry.hpp** — 기존 복사 + `regress_direction()` 추가:
- 최근 n개 점에 대한 PCA 방향벡터 계산 (2x2 공분산 행렬 고유벡터)
- chain 진행 방향과 같도록 부호 보정

**1-4. debug_publish.hpp** — 그대로 복사, namespace만 `chaining_mr_ver`로 변경

### Phase 2: 복사 모듈 (namespace만 변경) (4파일)

**2-1. safety_checker.hpp**
**2-2. magnetic_planner.hpp + magnetic_planner.cpp**
**2-3. path_postprocessor.hpp + path_postprocessor.cpp**

### Phase 3: 핵심 신규/수정 모듈 (3모듈)

**3-1. line_chainer.hpp + line_chainer.cpp** — 완전 신규

```
LineChainer::chain(all_candidates, params) → ChainResult
  ├── left seed 선택: y>0인 점 중 ego 최근접
  ├── right seed 선택: y<0인 점 중 ego 최근접
  ├── chain_one_side(all_candidates, left_seed, is_left=true, params) → left_chain
  │     (left seed에서 출발한 chain이 곧 좌측 corridor)
  ├── chain_one_side(all_candidates, right_seed, is_left=false, params) → right_chain
  │     (right seed에서 출발한 chain이 곧 우측 corridor)
  ├── resample_chain(left_chain, ds)
  └── resample_chain(right_chain, ds)
```

**visited 배열은 좌/우 chain이 공유한다. 모든 포인트가 반드시 좌 or 우 chain에 소속될 때까지 반복.**

핵심 알고리즘 `chain_one_side(all_candidates, seed_idx, is_left, visited, params)`:
1. Seed: chain()에서 전달받음. **y좌표는 seed 선택에만 사용되고, 이후 chaining은 y 무관**
   → 어느 seed에서 출발한 chain에 속하느냐가 그 점의 좌/우를 결정
2. 방향벡터: compute_direction() — 1점이면 ego→seed, 2+점이면 PCA regression
3. 탐색: radius r 내 + forward_angle 내, visited=false인 후보 수집
4. 후보 없으면 r += step으로 확장 (max까지)
5. Cone priority: cone 후보 있으면 lane 제거
6. Scoring: cross2(direction, to_candidate) 기반
   - Left chain: -cross → 오른쪽(도로 안쪽)일수록 높은 점수
   - Right chain: +cross → 왼쪽(도로 안쪽)일수록 높은 점수
7. 최고 점수 후보 선택, visited[i]=true 마킹, 반복
8. max radius까지 확장해도 후보 없으면 그 chain은 종료

chain() 흐름:
```
visited = [false] * N
left_chain  = chain_one_side(all, left_seed,  is_left=true,  visited, params)
right_chain = chain_one_side(all, right_seed, is_left=false, visited, params)
// visited 공유 → left가 잡은 점은 right에서 제외됨

// 노이즈 처리 (병렬적, 노이즈끼리 상속 금지):
// 참조 대상은 chaining DFS에서 방문된 원본 chain 포인트만 (노이즈 제외)
chain_pts = left_chain ∪ right_chain  // DFS로 확정된 점들만
for each unvisited point p:  // 각 노이즈는 독립적으로 판정
    nearest = chain_pts 중 p와 가장 가까운 점  // 다른 노이즈 참조 안 함
    nearest가 left_chain 소속 → p를 left_chain에 추가
    nearest가 right_chain 소속 → p를 right_chain에 추가
// 결과: 모든 점이 반드시 좌 or 우에 소속됨
```

리샘플 `resample_chain()`:
- cone-cone 구간 → CONE으로 리샘플
- 그 외 (lane-lane, lane-cone) → LANE으로 리샘플
- ds=0.1m 간격

**3-2. costmap_generator.hpp + costmap_generator.cpp** — 인터페이스 변경

기존: `generate(vector<Point2D> cones, vector<Point2D> lanes, params)`
변경: `generate(vector<ChainedPoint> left_chain, vector<ChainedPoint> right_chain, params)`

내부: 두 체인을 순회하며 ChainedPoint.type에 따라:
- CONE → apply_source(..., cone_cost_max, sigma, threshold, cone_radius)
- LANE → apply_source(..., lane_cost_max, sigma, threshold, 0.0)

apply_source()는 변경 없이 그대로 재사용.

**3-3. lc_planner_node.hpp + lc_planner_node.cpp** — 파이프라인 재구성

- parse_input(): 모든 cone/lane을 단일 vector<ChainedPoint>로 수집 (L/R 분리 없음)
  → cone은 PointType::CONE, lane은 PointType::LANE 태그만 붙임
  → 좌/우 구분은 LineChainer가 seed 선택(y>0/y<0)으로만 결정하고,
    이후 chaining된 소속으로 좌/우가 정해짐
- on_timer(): Stage 2-3에 LineChainer 호출 추가
- Debug publishers 추가: `/planning/debug/left_chain`, `/planning/debug/right_chain`
- 멤버 추가: `LineChainer line_chainer_`

### Phase 4: 빌드/설정 (4파일)

**4-1. CMakeLists.txt** — mr_ver 기반 + `planning_lc_chainer` 라이브러리 추가
- 4개 SHARED lib: chainer, costmap, planner, postprocess
- 1개 component: lc_planner_component (4개 링크)

**4-2. package.xml** — mr_ver 기반, 이름만 `chaining_mr_ver`

**4-3. planning_lc.yaml** — mr_ver 기반 + chainer 섹션 추가, 노드명 `lc_planner_node`

**4-4. planning_lc.launch.py** — mr_ver 기반, 패키지/노드명 변경

---

## 재사용 원본 파일 (planning_mr_ver)

| 파일 | 경로 | 변경 |
|------|------|------|
| types.hpp | `.../planning_mr_ver/include/.../common/types.hpp` | 확장 |
| params.hpp | `.../planning_mr_ver/include/.../common/params.hpp` | 확장 |
| geometry.hpp | `.../planning_mr_ver/include/.../common/geometry.hpp` | 확장 |
| debug_publish.hpp | `.../planning_mr_ver/include/.../common/debug_publish.hpp` | namespace만 |
| safety_checker.hpp | `.../planning_mr_ver/include/.../safety/safety_checker.hpp` | namespace만 |
| magnetic_planner.* | `.../planning_mr_ver/{include,src}/.../planner/magnetic_planner.*` | namespace만 |
| path_postprocessor.* | `.../planning_mr_ver/{include,src}/.../postprocess/path_postprocessor.*` | namespace만 |
| costmap_generator.* | `.../planning_mr_ver/{include,src}/.../costmap/costmap_generator.*` | 인터페이스 변경 |
| mr_planner_node.* | `.../planning_mr_ver/{include,src}/.../nodes/mr_planner_node.*` | 파이프라인 재구성 |

---

## 검증

1. **빌드**: `cd ~/ev_ws/planning && colcon build --packages-select chaining_mr_ver`
2. **실행**: `ros2 launch chaining_mr_ver planning_lc.launch.py`
3. **ROSbag 테스트**: 기존 rosbag으로 재생하며 RViz2에서 시각화
4. **확인 항목**:
   - `/planning/debug/left_chain`, `/planning/debug/right_chain`: 연속 폴리라인 형성
   - `/planning/debug/costmap`: 틈 없는 연속 자기장 벽
   - `/planning/path`: 경로가 두 벽 사이를 안정적으로 통과
5. **A/B 비교**: 동일 rosbag으로 mr_ver vs lc_ver 결과 비교
