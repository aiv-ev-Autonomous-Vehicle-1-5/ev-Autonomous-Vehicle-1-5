/**
 * @file mock_lane_node.cpp
 * @brief 하드코딩 직선 차선 경계 퍼블리셔 + RViz2 디버그 Marker 발행
 *
 * 카메라 차선 인식 노드가 준비되지 않은 상태에서 planning 파이프라인을
 * 테스트하기 위한 mock 노드.
 *
 * 발행 토픽:
 *   /perception/lane_boundaries  (ev_msgs/LaneBoundaryArray, BestEffort)
 *
 * 차선 배치 (base_link 기준):
 *   - 왼쪽 경계: y = +lane_half_width, x = [lane_x_start, lane_x_end]
 *   - 오른쪽 경계: y = -lane_half_width
 *   - 점 간격: point_spacing [m]
 *
 * 파라미터는 config/params.yaml 로 설정 가능.
 */

#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ev_msgs/msg/lane_boundary.hpp>
#include <ev_msgs/msg/lane_boundary_array.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

class MockLanePublisher : public rclcpp::Node
{
public:
  MockLanePublisher()
  : Node("mock_lane_publisher")
  {
    // ── 파라미터 선언 ──
    this->declare_parameter("lane_half_width", 1.5);
    this->declare_parameter("lane_x_start", 0.5);
    this->declare_parameter("lane_x_end", 5.0);
    this->declare_parameter("point_spacing", 0.5);
    this->declare_parameter("publish_hz", 10.0);

    half_w_   = this->get_parameter("lane_half_width").as_double();
    x_start_  = this->get_parameter("lane_x_start").as_double();
    x_end_    = this->get_parameter("lane_x_end").as_double();
    spacing_  = this->get_parameter("point_spacing").as_double();
    double hz = this->get_parameter("publish_hz").as_double();

    // ── 정적 차선 포인트 생성 ──
    left_pts_  = make_line(+half_w_);
    right_pts_ = make_line(-half_w_);

    // ── Publisher: lane boundaries (BestEffort, depth=1) ──
    auto qos_be = rclcpp::QoS(1).best_effort();
    lane_pub_ = this->create_publisher<ev_msgs::msg::LaneBoundaryArray>(
      "/perception/lane_boundaries", qos_be);

    // ── Timer ──
    auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockLanePublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
      "Mock lane publisher started — left y=+%.1fm, right y=-%.1fm, "
      "x=[%.1f, %.1f]m, %zu pts/side, %.0f Hz",
      half_w_, half_w_, x_start_, x_end_, left_pts_.size(), hz);
  }

private:
  // ── 직선 차선 포인트 생성 (가까운 점 → 먼 점 순서) ──
  std::vector<geometry_msgs::msg::Point> make_line(double y_offset)
  {
    std::vector<geometry_msgs::msg::Point> pts;
    for (double x = x_start_; x <= x_end_ + 1e-6; x += spacing_) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y_offset;
      p.z = 0.0;
      pts.push_back(p);
    }
    return pts;
  }

  void on_timer()
  {
    auto now = this->get_clock()->now();

    // ── LaneBoundaryArray 발행 ──
    ev_msgs::msg::LaneBoundaryArray lane_msg;
    lane_msg.header.stamp = now;
    lane_msg.header.frame_id = "base_link";

    ev_msgs::msg::LaneBoundary left;
    left.header = lane_msg.header;
    left.points = left_pts_;
    left.confidence = 1.0f;

    ev_msgs::msg::LaneBoundary right;
    right.header = lane_msg.header;
    right.points = right_pts_;
    right.confidence = 1.0f;

    lane_msg.boundaries.push_back(left);
    lane_msg.boundaries.push_back(right);
    lane_pub_->publish(lane_msg);
  }

  // ── 멤버 변수 ──
  double half_w_, x_start_, x_end_, spacing_;
  std::vector<geometry_msgs::msg::Point> left_pts_;
  std::vector<geometry_msgs::msg::Point> right_pts_;

  rclcpp::Publisher<ev_msgs::msg::LaneBoundaryArray>::SharedPtr lane_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockLanePublisher>());
  rclcpp::shutdown();
  return 0;
}
