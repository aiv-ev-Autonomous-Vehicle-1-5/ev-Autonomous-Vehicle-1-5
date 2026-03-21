/**
 * @file mock_lane_node.cpp
 * @brief 직선/커브 차선 경계 퍼블리셔 — planning 파이프라인 테스트용 mock 노드
 *
 * 발행 토픽:
 *   /perception/lane_boundaries  (ev_msgs/LaneBoundaryArray, BestEffort)
 *
 * shape 파라미터로 직선/커브 선택:
 *   "straight" : 기존 직선 모드 (y = ±lane_half_width)
 *   "curve"    : 원호(circular arc) 커브 모드 (좌회전)
 *
 * [커브 기하학 — base_link 기준]
 *   - 차량이 원점에서 +x 방향으로 출발
 *   - straight_length [m] 직진 후 좌회전 커브 시작
 *   - 곡률 중심: (curve_start_x, curve_radius)
 *   - 경계점 φ ∈ [0, arc_rad]:
 *       r_eff = curve_radius - y_offset   (좌: 내측, 우: 외측)
 *       x = curve_start_x + r_eff * sin(φ)
 *       y = curve_radius  - r_eff * cos(φ)
 *   - φ=0 에서 y = curve_radius - r_eff = y_offset → 직선 끝과 연속 ✓
 *
 * 파라미터 (config/params.yaml):
 *   shape           : "straight" | "curve"
 *   lane_half_width : 중앙~경계 횡방향 거리 [m]
 *   lane_x_start    : 차선 시작 x [m]
 *   lane_x_end      : 직선 모드 차선 끝 x [m]
 *   point_spacing   : 점 간격 [m]
 *   publish_hz      : 발행 주기 [Hz]
 *   curve_radius    : (커브 모드) 중심선 곡률 반경 [m]
 *   curve_arc_deg   : (커브 모드) 커브 호 각도 [deg]
 *   straight_length : (커브 모드) 커브 전 직선 구간 길이 [m]
 */

#include <chrono>
#include <cmath>
#include <string>
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
    this->declare_parameter("shape",           std::string("straight"));
    this->declare_parameter("lane_half_width", 1.5);
    this->declare_parameter("lane_x_start",    0.5);
    this->declare_parameter("lane_x_end",      5.0);
    this->declare_parameter("point_spacing",   0.5);
    this->declare_parameter("publish_hz",      10.0);
    this->declare_parameter("curve_radius",    5.0);
    this->declare_parameter("curve_arc_deg",   90.0);
    this->declare_parameter("straight_length", 1.0);

    shape_         = this->get_parameter("shape").as_string();
    half_w_        = this->get_parameter("lane_half_width").as_double();
    x_start_       = this->get_parameter("lane_x_start").as_double();
    x_end_         = this->get_parameter("lane_x_end").as_double();
    spacing_       = this->get_parameter("point_spacing").as_double();
    double hz      = this->get_parameter("publish_hz").as_double();
    curve_radius_  = this->get_parameter("curve_radius").as_double();
    curve_arc_deg_ = this->get_parameter("curve_arc_deg").as_double();
    straight_len_  = this->get_parameter("straight_length").as_double();

    // ── 정적 차선 포인트 생성 ──
    if (shape_ == "curve") {
      left_pts_  = make_curve(+half_w_);
      right_pts_ = make_curve(-half_w_);
      RCLCPP_INFO(this->get_logger(),
        "Mock lane publisher started [CURVE] — half_w=%.1fm, R=%.1fm, arc=%.0fdeg, "
        "straight=%.1fm, %zu/%zu pts(L/R), %.0f Hz",
        half_w_, curve_radius_, curve_arc_deg_, straight_len_,
        left_pts_.size(), right_pts_.size(), hz);
    } else {
      left_pts_  = make_line(+half_w_);
      right_pts_ = make_line(-half_w_);
      RCLCPP_INFO(this->get_logger(),
        "Mock lane publisher started [STRAIGHT] — left y=+%.1fm, right y=-%.1fm, "
        "x=[%.1f, %.1f]m, %zu pts/side, %.0f Hz",
        half_w_, half_w_, x_start_, x_end_, left_pts_.size(), hz);
    }

    // ── Publisher: lane boundaries (BestEffort, depth=1) ──
    auto qos_be = rclcpp::QoS(1).best_effort();
    lane_pub_ = this->create_publisher<ev_msgs::msg::LaneBoundaryArray>(
      "/perception/lane_boundaries", qos_be);

    // ── Timer ──
    auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockLanePublisher::on_timer, this));
  }

private:
  // ── 직선 차선 포인트 생성 ──
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

  // ── 커브 차선 포인트 생성 (좌회전 원호) ──
  //
  // 곡률 중심: (curve_start_x, curve_radius)
  // 이 경계의 실효 반경: r_eff = curve_radius - y_offset
  //   y_offset > 0 (좌측 경계) → 내측 원호 (r_eff 작음)
  //   y_offset < 0 (우측 경계) → 외측 원호 (r_eff 큼)
  // φ=0 에서 x=curve_start_x, y=y_offset → 직선 구간과 연속
  std::vector<geometry_msgs::msg::Point> make_curve(double y_offset)
  {
    std::vector<geometry_msgs::msg::Point> pts;

    // 1) 직선 구간: x_start ~ x_start + straight_len
    double curve_start_x = x_start_ + straight_len_;
    for (double x = x_start_; x < curve_start_x - 1e-6; x += spacing_) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y_offset;
      p.z = 0.0;
      pts.push_back(p);
    }

    // 2) 원호 구간
    double arc_rad = curve_arc_deg_ * M_PI / 180.0;
    double r_eff   = curve_radius_ - y_offset;

    if (r_eff <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "make_curve: r_eff=%.2f <= 0 (radius=%.1f, y_offset=%.1f). 포인트 생략.",
        r_eff, curve_radius_, y_offset);
      return pts;
    }

    // 호 길이 기준으로 φ 증분 계산
    double d_phi = spacing_ / r_eff;

    for (double phi = 0.0; phi <= arc_rad + 1e-6; phi += d_phi) {
      geometry_msgs::msg::Point p;
      p.x = curve_start_x + r_eff * std::sin(phi);
      p.y = curve_radius_  - r_eff * std::cos(phi);
      p.z = 0.0;
      pts.push_back(p);
    }

    return pts;
  }

  void on_timer()
  {
    auto now = this->get_clock()->now();

    ev_msgs::msg::LaneBoundaryArray lane_msg;
    lane_msg.header.stamp    = now;
    lane_msg.header.frame_id = "base_link";

    ev_msgs::msg::LaneBoundary left;
    left.header     = lane_msg.header;
    left.points     = left_pts_;
    left.confidence = 1.0f;

    ev_msgs::msg::LaneBoundary right;
    right.header     = lane_msg.header;
    right.points     = right_pts_;
    right.confidence = 1.0f;

    lane_msg.boundaries.push_back(left);
    lane_msg.boundaries.push_back(right);
    lane_pub_->publish(lane_msg);
  }

  // ── 멤버 변수 ──
  std::string shape_;
  double half_w_, x_start_, x_end_, spacing_;
  double curve_radius_, curve_arc_deg_, straight_len_;

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
