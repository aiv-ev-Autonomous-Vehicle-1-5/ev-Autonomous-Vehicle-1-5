// ============================================================================
// debug_visualizer.cpp — RViz2 디버그 시각화 구현
// ============================================================================

#include "pp_controller_cpp/debug/debug_visualizer.hpp"

#include <cmath>

namespace pp_controller_cpp
{

DebugVisualizer::DebugVisualizer(MarkerPub lookahead_pub, MarkerPub arc_pub)
  : lookahead_pub_(std::move(lookahead_pub))
  , arc_pub_(std::move(arc_pub))
{
}

// ---------------------------------------------------------------------------
// publish_lookahead_point: lookahead 목표점을 초록색 SPHERE로 발행
// ---------------------------------------------------------------------------
// PP가 선택한 목표점의 위치를 base_link 기준으로 시각화.
// 구독자가 없으면 마커를 생성하지 않는다 (lazy).
// ---------------------------------------------------------------------------
void DebugVisualizer::publish_lookahead_point(
  double tx, double ty, const rclcpp::Time & stamp)
{
  if (lookahead_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "base_link";
  m.header.stamp = stamp;
  m.ns = "pp_debug";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = tx;
  m.pose.position.y = ty;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  lookahead_pub_->publish(m);
}

// ---------------------------------------------------------------------------
// publish_pursuit_arc: PP 원호 궤적을 노란색 LINE_STRIP으로 발행
// ---------------------------------------------------------------------------
// kappa_pp로부터 차량이 따라갈 예상 원호를 샘플링하여 시각화.
// kappa_pp ≈ 0 (직선)이면 직선 보간.
//
// 원호 기하학:
//   R = 1/kappa_pp, 회전 중심 (0, R)
//   kappa_pp > 0: 좌회전 (반시계), kappa_pp < 0: 우회전 (시계)
// ---------------------------------------------------------------------------
void DebugVisualizer::publish_pursuit_arc(
  double tx, double ty, double kappa_pp, const rclcpp::Time & stamp)
{
  if (arc_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::Marker arc;
  arc.header.frame_id = "base_link";
  arc.header.stamp = stamp;
  arc.ns = "pp_debug";
  arc.id = 1;
  arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
  arc.action = visualization_msgs::msg::Marker::ADD;
  arc.pose.orientation.w = 1.0;
  arc.scale.x = 0.03;  // 선 두께
  arc.color.r = 1.0f;
  arc.color.g = 1.0f;
  arc.color.b = 0.0f;
  arc.color.a = 0.8f;

  constexpr int N_ARC = 30;  // 원호 샘플 수

  if (std::abs(kappa_pp) < 1e-6) {
    // 직선: 차량 원점 → 목표점까지 직선 보간
    for (int i = 0; i <= N_ARC; ++i) {
      const double t = static_cast<double>(i) / N_ARC;
      geometry_msgs::msg::Point p;
      p.x = tx * t;
      p.y = ty * t;
      p.z = 0.0;
      arc.points.push_back(p);
    }
  } else {
    // 원호: 회전 중심 (cx, cy) = (0, R), 반지름 |R|
    // R = 1/kappa_pp (좌회전: R>0, 우회전: R<0)
    const double R = 1.0 / kappa_pp;
    const double cx = 0.0;
    const double cy = R;
    const double abs_R = std::abs(R);

    // 시작각: 차량 원점(0,0)에서의 각도 = atan2(0 - cy, 0 - cx)
    const double theta_start = std::atan2(-cy, -cx);
    // 종료각: 목표점(tx, ty)에서의 각도 = atan2(ty - cy, tx - cx)
    const double theta_end = std::atan2(ty - cy, tx - cx);

    // 각도 차이 계산 (회전 방향 고려)
    double dtheta = theta_end - theta_start;
    // kappa_pp > 0 (좌회전): 반시계 방향 → dtheta > 0이어야 함
    // kappa_pp < 0 (우회전): 시계 방향 → dtheta < 0이어야 함
    if (kappa_pp > 0.0) {
      while (dtheta < 0.0) dtheta += 2.0 * M_PI;
      while (dtheta > 2.0 * M_PI) dtheta -= 2.0 * M_PI;
    } else {
      while (dtheta > 0.0) dtheta -= 2.0 * M_PI;
      while (dtheta < -2.0 * M_PI) dtheta += 2.0 * M_PI;
    }

    for (int i = 0; i <= N_ARC; ++i) {
      const double t = static_cast<double>(i) / N_ARC;
      const double theta = theta_start + dtheta * t;
      geometry_msgs::msg::Point p;
      p.x = cx + abs_R * std::cos(theta);
      p.y = cy + abs_R * std::sin(theta);
      p.z = 0.0;
      arc.points.push_back(p);
    }
  }

  arc_pub_->publish(arc);
}

}  // namespace pp_controller_cpp
