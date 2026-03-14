/**
 * @file chain_resampler.cpp
 * @brief DirectionChainer — Component 리샘플링 구현
 *
 * [포함 함수]
 *   - resample_component(): backbone + branch의 모든 edge를 resample_ds 간격으로 보간
 *     - resample_edge 람다에 is_bb 파라미터를 받아 backbone/branch를 구분
 *     - backbone edge → is_backbone=true로 마킹하여 costmap에서 bbox_cost_max 적용 보장
 *     - branch edge  → is_backbone=false로 마킹하여 원래 타입 기반 비용 적용
 *
 * [의존 관계]
 *   - types.hpp: ChainPoint, BranchInfo, PointType
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <cmath>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// Component 리샘플링 — 전 edge 보간
// ============================================================================
//
// backbone + branch의 모든 edge를 resample_ds 간격으로 선형 보간하여
// 균등한 점열을 생성한다. costmap에 연속적인 비용 장벽을 보장.
//
// 보간점의 type: 양끝 모두 BBOX일 때만 BBOX, 혼합 edge는 LANE
// 전환 edge(lane↔bbox)에서는 LANE으로 처리하여
// costmap에서 해당 구간이 lane_cost_max로 적용된다.
//
std::vector<ChainPoint> DirectionChainer::resample_component(
  const std::vector<ChainPoint> & points,
  const std::vector<int> & backbone_ids,
  const std::vector<BranchInfo> & branches,
  double resample_ds) const
{
  std::vector<ChainPoint> resampled;
  if (backbone_ids.empty()) return resampled;

  // edge 보간 헬퍼: 시작점 a는 추가하지만, 끝점 b는 추가하지 않음
  // is_bb: true이면 backbone edge → 보간점에 is_backbone=true 설정
  auto resample_edge = [&](const ChainPoint & a, const ChainPoint & b, bool is_bb) {
    ChainPoint a_copy = a;
    a_copy.is_backbone = is_bb;
    resampled.push_back(a_copy);

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len >= resample_ds) {
      const PointType seg_type =
        (a.type == PointType::BBOX && b.type == PointType::BBOX)
          ? PointType::BBOX : PointType::LANE;

      for (double d = resample_ds; d < len; d += resample_ds) {
        const double t = d / len;
        ChainPoint interp;
        interp.x = a.x + t * dx;
        interp.y = a.y + t * dy;
        interp.type = seg_type;
        interp.is_backbone = is_bb;
        resampled.push_back(interp);
      }
    }
  };

  // 1) backbone edges 리샘플 (is_backbone=true)
  for (size_t i = 0; i + 1 < backbone_ids.size(); ++i) {
    resample_edge(points[backbone_ids[i]], points[backbone_ids[i + 1]], true);
  }
  if (!backbone_ids.empty()) {
    ChainPoint last = points[backbone_ids.back()];
    last.is_backbone = true;
    resampled.push_back(last);
  }

  // 2) 각 branch의 연결 edge + 내부 edges 리샘플
  for (const auto & branch : branches) {
    if (branch.points.empty()) continue;
    if (branch.parent_backbone_idx < 0 ||
        branch.parent_backbone_idx >= static_cast<int>(backbone_ids.size())) {
      continue;
    }

    const auto & parent_pt = points[backbone_ids[branch.parent_backbone_idx]];
    resample_edge(parent_pt, branch.points[0], false);

    for (size_t i = 0; i + 1 < branch.points.size(); ++i) {
      resample_edge(branch.points[i], branch.points[i + 1], false);
    }
    resampled.push_back(branch.points.back());
  }

  return resampled;
}

}  // namespace chaining_costmap_ver
