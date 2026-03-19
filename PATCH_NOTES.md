# Patch Notes

## 2026-03-20

### [Feature] backbone 양방향 chaining (backward + forward)

- **파일**: `planning/chaining_costmap_ver/src/chainer/backbone_extractor.cpp`
- **변경**: `extract_backbone()`이 seed에서 전방(+x)만 체이닝하던 것을 후방(-x) + 전방(+x) 양방향으로 확장
  - 결과 backbone: reverse(backward) + [seed] + forward
  - 전방/후방 각각 max_chain_len - 1 개까지 확장 가능
  - visited_set 공유로 중복 방문 방지

### [BugFix] backbone 교차 판정 owner 덮어쓰기 버그 수정

- **파일**: `planning/chaining_costmap_ver/src/chainer/direction_chainer.cpp`
- **문제**: left backbone이 right seed를 체이닝한 교차 상태를 감지하는 로직이 2단계(right backbone) 빌드 **이후**에 있어서, `owner[right_seed]`가 `RIGHT_BACKBONE`으로 덮어써져 교차 판정이 항상 실패
- **수정**: 교차 판정을 각 backbone 빌드 직후로 이동 — left 교차 판정은 1단계 직후, right 교차 판정은 2단계 직후에 수행

### [Feature] backbone 교차 시 local_goal 중간점 폴백

- **파일**: `direction_chainer.cpp`, `goal_calculator.cpp`, `chain_types.hpp`
- **문제**: 한쪽 backbone이 반대쪽 seed까지 체이닝하면 양쪽 backbone 끝점 기반 goal 계산이 왜곡됨
- **수정**:
  - `DirectionChainResult`에 `left_crossed_right` / `right_crossed_left` 교차 플래그 추가
  - `direction_chainer.cpp`에서 `owner[right_seed] == LEFT_BACKBONE` 검사로 교차 판정
  - `goal_calculator.cpp`에서 교차 감지 시 해당 backbone의 인덱스 중간점을 local_goal로 사용

### [Feature] make_bbox 클러스터 분할 (K-means)

- **파일**: `make_bbox_node.hpp`, `make_bbox_node.cpp`, `make_bbox_params.yaml`
- **문제**: 가까이 붙어있는 라바콘이 DBSCAN에서 하나의 클러스터로 합쳐짐
- **수정**: bbox 생성 전 oversized 클러스터를 K-means로 분할
  - XY 최장축 > `split_cone_diameter_m`(0.5m)이면 `k = round(최장축 / 콘지름)`으로 분할
  - 분할 후 `split_min_points`(5) 미만인 서브클러스터는 제거

### [BugFix] chaining backbone에서 bbox가 knn k개 제한에 밀려 후보 탈락하는 버그 수정

- **파일**: `planning/chaining_costmap_ver/src/chainer/backbone_extractor.cpp`
- **문제**: `chain_one_direction()`에서 knn(k=40)이 타입 구분 없이 거리순 k개를 뽑기 때문에, lane point가 많으면 가까이 있는 bbox도 k개 후보에 포함되지 않아 라바콘이 무시됨
- **수정**: 2-phase BBOX 최우선 탐색으로 변경
  - Phase 1: d_max 범위 내 모든 bbox를 knn 없이 직접 전수 탐색 → G2+G3 게이트 적용
  - Phase 2 (fallback): bbox 후보가 없거나 모두 게이트 탈락 시에만 기존 knn(k) → 게이트 → bbox-first 선택
- **문서 반영**: `planning/chaining_costmap_ver/PIPELINE.md` 업데이트
