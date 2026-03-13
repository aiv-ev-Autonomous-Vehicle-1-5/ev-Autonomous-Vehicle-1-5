/**
 * @file chain_resampler.cpp
 * @brief DirectionChainer — Component 리샘플링 구현
 *
 * [포함 함수]
 *   - resample_component(): backbone + branch의 모든 edge를 resample_ds 간격으로 보간
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
// 보간점의 type: 양끝이 모두 CONE이면 CONE, 아니면 LANE
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
  auto resample_edge = [&](const ChainPoint & a, const ChainPoint & b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::sqrt(dx * dx + dy * dy);

    resampled.push_back(a);

    if (len >= resample_ds) {
      const PointType seg_type =
        (a.type == PointType::CONE && b.type == PointType::CONE)
          ? PointType::CONE : PointType::LANE;

      for (double d = resample_ds; d < len; d += resample_ds) {
        const double t = d / len;
        ChainPoint interp;
        interp.x = a.x + t * dx;
        interp.y = a.y + t * dy;
        interp.type = seg_type;
        resampled.push_back(interp);
      }
    }
  };

  // 1) backbone edges 리샘플
  for (size_t i = 0; i + 1 < backbone_ids.size(); ++i) {
    resample_edge(points[backbone_ids[i]], points[backbone_ids[i + 1]]);
  }
  if (!backbone_ids.empty()) {
    resampled.push_back(points[backbone_ids.back()]);
  }

  // 2) 각 branch의 연결 edge + 내부 edges 리샘플
  for (const auto & branch : branches) {
    if (branch.points.empty()) continue;
    if (branch.parent_backbone_idx < 0 ||
        branch.parent_backbone_idx >= static_cast<int>(backbone_ids.size())) {
      continue;
    }

    const auto & parent_pt = points[backbone_ids[branch.parent_backbone_idx]];
    resample_edge(parent_pt, branch.points[0]);

    for (size_t i = 0; i + 1 < branch.points.size(); ++i) {
      resample_edge(branch.points[i], branch.points[i + 1]);
    }
    resampled.push_back(branch.points.back());
  }

  return resampled;
}

}  // namespace chaining_costmap_ver
