# Patch Notes

## 2026-03-20

### [BugFix] chaining backbone에서 bbox가 knn k개 제한에 밀려 후보 탈락하는 버그 수정

- **파일**: `planning/chaining_costmap_ver/src/chainer/backbone_extractor.cpp`
- **문제**: `chain_one_direction()`에서 knn(k=40)이 타입 구분 없이 거리순 k개를 뽑기 때문에, lane point가 많으면 가까이 있는 bbox도 k개 후보에 포함되지 않아 라바콘이 무시됨
- **수정**: 2-phase BBOX 최우선 탐색으로 변경
  - Phase 1: d_max 범위 내 모든 bbox를 knn 없이 직접 전수 탐색 → G2+G3 게이트 적용
  - Phase 2 (fallback): bbox 후보가 없거나 모두 게이트 탈락 시에만 기존 knn(k) → 게이트 → bbox-first 선택
- **문서 반영**: `planning/chaining_costmap_ver/PIPELINE.md` 업데이트
