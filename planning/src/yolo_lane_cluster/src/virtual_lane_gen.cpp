/**
 * @file virtual_lane_gen.cpp
 * @brief 가상 차선 생성 — 실제 차선에서 track_width(1.5m) 안쪽 오프셋
 *
 * 각 point의 방향벡터(tangent)를 구하고,
 * 수직 방향(normal)으로 track_width만큼 이동하여 가상 반대편 차선을 생성한다.
 *
 * 오프셋 방향:
 *   - 왼쪽 차선(LEFT)  → 오른쪽(안쪽)으로 오프셋: normal = ( ty, -tx)
 *   - 오른쪽 차선(RIGHT) → 왼쪽(안쪽)으로 오프셋: normal = (-ty,  tx)
 *
 * 좌표계 (ego, base_link 기준):
 *   x = 전방(+), y = 좌측(+)
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

#include <cmath>

namespace yolo_lane_cluster
{

ev_msgs::msg::LaneBoundary YoloLaneClusterNode::generate_virtual_lane(
  const ev_msgs::msg::LaneBoundary & real_lane,
  LaneSide real_side) const
{
  ev_msgs::msg::LaneBoundary vl;
  vl.header     = real_lane.header;
  vl.confidence = real_lane.confidence;
  vl.lane_id    = -1;  // 가상 차선 표시

  const auto & pts = real_lane.points;
  if (pts.size() < 2) return vl;

  const double offset = params_.track_width;

  for (size_t i = 0; i < pts.size(); ++i) {
    // ── tangent 계산 (중심 차분, 양 끝은 편측 차분) ──
    double tx, ty;
    if (i == 0) {
      tx = pts[1].x - pts[0].x;
      ty = pts[1].y - pts[0].y;
    } else if (i == pts.size() - 1) {
      tx = pts[i].x - pts[i - 1].x;
      ty = pts[i].y - pts[i - 1].y;
    } else {
      tx = pts[i + 1].x - pts[i - 1].x;
      ty = pts[i + 1].y - pts[i - 1].y;
    }

    // 정규화
    double len = std::sqrt(tx * tx + ty * ty);
    if (len < 1e-9) continue;
    tx /= len;
    ty /= len;

    // ── normal: tangent에 수직, 안쪽 방향 ──
    double nx, ny;
    if (real_side == LaneSide::LEFT) {
      // 왼쪽 차선 → 오른쪽(y 감소)으로 오프셋
      nx =  ty;
      ny = -tx;
    } else {
      // 오른쪽 차선 → 왼쪽(y 증가)으로 오프셋
      nx = -ty;
      ny =  tx;
    }

    geometry_msgs::msg::Point vpt;
    vpt.x = pts[i].x + nx * offset;
    vpt.y = pts[i].y + ny * offset;
    vpt.z = 0.0;
    vl.points.push_back(vpt);
  }

  return vl;
}

}  // namespace yolo_lane_cluster
