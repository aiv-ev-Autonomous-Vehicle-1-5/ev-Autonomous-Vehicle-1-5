// ============================================================================
// debug_visualizer.hpp — RViz2 디버그 시각화
// ============================================================================
// Pure Pursuit 디버그용 RViz2 마커를 생성/발행한다.
//
// 발행 마커:
//   1) lookahead_point (SPHERE) — PP가 선택한 목표점을 초록색 구로 표시
//   2) pursuit_arc (LINE_STRIP) — PP 곡률로부터 예상 원호 궤적을 노란색 선으로 표시
//
// 구독자가 없으면 마커를 생성하지 않는다 (lazy publishing).
// ============================================================================

#ifndef PP_CONTROLLER_CPP__DEBUG__DEBUG_VISUALIZER_HPP_
#define PP_CONTROLLER_CPP__DEBUG__DEBUG_VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pp_controller_cpp
{

class DebugVisualizer
{
public:
  using MarkerPub = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;

  /// 생성자: lookahead_point와 pursuit_arc 퍼블리셔를 주입받는다
  DebugVisualizer(MarkerPub lookahead_pub, MarkerPub arc_pub);

  /// lookahead 목표점을 초록색 SPHERE 마커로 발행 (lazy)
  void publish_lookahead_point(double tx, double ty, const rclcpp::Time & stamp);

  /// PP 곡률로부터 예상 원호 궤적을 노란색 LINE_STRIP으로 발행 (lazy)
  /// kappa_pp ≈ 0 이면 직선 보간, 아니면 원호 샘플링
  void publish_pursuit_arc(
    double tx, double ty, double kappa_pp, const rclcpp::Time & stamp);

private:
  MarkerPub lookahead_pub_;
  MarkerPub arc_pub_;
};

}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__DEBUG__DEBUG_VISUALIZER_HPP_
