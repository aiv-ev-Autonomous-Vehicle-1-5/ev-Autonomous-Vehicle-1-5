/**
 * @file input_parser.cpp
 * @brief [Stage 1] bbox/차선 ROS 메시지 → ChainPoint 벡터 변환 구현
 *
 * BBox(LiDAR 장애물)와 LaneBoundary(카메라 차선)를
 * 단일 ChainPoint 벡터로 변환한다.
 * 좌표 변환은 노드의 on_timer()에서 tf2로 처리 (velodyne→base_link).
 *
 * [의존 관계]
 *   - input_parser.hpp: parse_input() 선언
 *   - types.hpp: ChainPoint, PointType
 */
#include "chaining_costmap_ver/nodes/input_parser.hpp"

namespace chaining_costmap_ver
{

void parse_input(
  const ev_msgs::msg::BBoxArray * bboxes,
  const ev_msgs::msg::LaneBoundaryArray * lanes,
  std::vector<ChainPoint> & all_pts)
{
  // BBox 수집 (velodyne 프레임 — on_timer()에서 tf2로 base_link 변환)
  if (bboxes) {
    for (const auto & b : bboxes->bboxes) {
      ChainPoint cp;
      cp.x = b.position.x;
      cp.y = b.position.y;
      cp.type = PointType::BBOX;
      cp.label = b.label;
      cp.size_x = b.size_x;
      cp.size_y = b.size_y;
      all_pts.push_back(cp);
    }
  }

  // 차선 점 수집 (이미 base_link 기준)
  if (lanes) {
    for (const auto & bd : lanes->boundaries) {
      for (const auto & p : bd.points) {
        ChainPoint cp;
        cp.x = p.x;
        cp.y = p.y;
        cp.type = PointType::LANE;
        cp.label = -1;
        all_pts.push_back(cp);
      }
    }
  }
}

}  // namespace chaining_costmap_ver
