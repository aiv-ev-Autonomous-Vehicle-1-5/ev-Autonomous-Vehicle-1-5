#include <cmath>        // sqrt, atan 등 수학 함수
#include <memory>       // shared_ptr
#include <string>       // string
#include <vector>       // vector
#include <algorithm>    // clamp

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "erp42_msgs/msg/control_command.hpp"

using std::placeholders::_1;

// ---------------------------------------------
// 2차원 벡터 길이 계산 함수
// 예: (x, y) 점까지의 거리 = sqrt(x^2 + y^2)
// ---------------------------------------------
static double norm2d(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

class PurePursuitRelativeNode : public rclcpp::Node
{
public:
  PurePursuitRelativeNode() : Node("pure_pursuit_relative_node")
  {
    // =============================================
    // 1. 파라미터 선언
    // =============================================
    // path_topic:
    //   차량 기준(base_link) 상대좌표 path를 받는 토픽
    //
    // cmd_topic:
    //   ERP42 제어 명령을 내보내는 토픽
    //
    // wheelbase:
    //   차량 축간거리
    //
    // lookahead:
    //   얼마나 앞의 점을 목표점으로 잡을지
    //
    // speed:
    //   목표 속도
    //
    // delta_max:
    //   최대 조향각(rad)
    //
    // brake_stop / brake_run:
    //   정지 시 / 주행 시 브레이크 값
    //
    // path_timeout_sec:
    //   path가 너무 오래 안 들어오면 정지
    //
    // min_x_target:
    //   목표점이 너무 가까이 있거나 뒤쪽이면 정지하기 위한 안전 기준
    this->declare_parameter<std::string>("path_topic", "/planning/path");
    this->declare_parameter<std::string>("cmd_topic",  "/erp42/control_command");

    this->declare_parameter<double>("wheelbase", 0.74);
    this->declare_parameter<double>("lookahead", 1.2);
    this->declare_parameter<double>("speed", 0.3);
    this->declare_parameter<double>("delta_max", 0.314);   // 약 18도
    this->declare_parameter<int>("brake_stop", 30);        // 0 ~ 150
    this->declare_parameter<int>("brake_run", 0);
    this->declare_parameter<double>("path_timeout_sec", 0.5);
    this->declare_parameter<double>("min_x_target", 0.05); // 목표점이 너무 뒤/가까우면 정지

    // =============================================
    // 2. 파라미터 읽기
    // =============================================
    path_topic_ = this->get_parameter("path_topic").as_string();
    cmd_topic_  = this->get_parameter("cmd_topic").as_string();

    L_ = this->get_parameter("wheelbase").as_double();
    Ld_default_ = this->get_parameter("lookahead").as_double();
    v_ = this->get_parameter("speed").as_double();
    delta_max_ = this->get_parameter("delta_max").as_double();
    brake_stop_ = this->get_parameter("brake_stop").as_int();
    brake_run_ = this->get_parameter("brake_run").as_int();
    path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
    min_x_target_ = this->get_parameter("min_x_target").as_double();

    // =============================================
    // 3. Subscriber / Publisher 생성
    // =============================================
    // path는 nav_msgs/Path 형식이라고 가정
    // 단, 중요한 건 각 점이 "base_link 기준 상대좌표"여야 한다는 점
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      10,
      std::bind(&PurePursuitRelativeNode::on_path, this, _1)
    );

    cmd_pub_ = this->create_publisher<erp42_msgs::msg::ControlCommand>(
      cmd_topic_,
      10
    );

    // =============================================
    // 4. 제어 루프 타이머
    // =============================================
    // 20Hz = 50ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitRelativeNode::on_timer, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "[PP Relative] path=%s cmd=%s L=%.2f Ld=%.2f v=%.2f delta_max=%.3f",
      path_topic_.c_str(),
      cmd_topic_.c_str(),
      L_,
      Ld_default_,
      v_,
      delta_max_
    );
  }

private:
  // =============================================
  // path 콜백
  // =============================================
  // 최신 path 저장
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = *msg;
    last_path_time_ = this->now();
  }

  // =============================================
  // path가 신선한지 확인
  // =============================================
  // path가 비어 있지 않고,
  // 마지막 수신 후 너무 오래 지나지 않았으면 true
  bool path_fresh() const
  {
    if (latest_path_.poses.empty()) {
      return false;
    }

    const double dt = (this->now() - last_path_time_).seconds();
    return dt <= path_timeout_sec_;
  }

  // =============================================
  // 정지 명령 발행
  // =============================================
  void publish_stop()
  {
    erp42_msgs::msg::ControlCommand cmd;
    cmd.speed = 0.0;
    cmd.steering = 0.0;
    cmd.brake = static_cast<uint8_t>(std::clamp(brake_stop_, 0, 150));
    cmd_pub_->publish(cmd);
  }

  // =============================================
  // 상대좌표 path에서 lookahead 목표점 찾기
  // =============================================
  //
  // 여기서는 path가 이미 차량 기준(base_link 기준)이라고 가정한다.
  // 즉,
  //   차량 위치 = (0, 0)
  //   차량 진행방향 = +x
  //
  // 방법:
  // 1) 원점에서 가장 가까운 path 점을 찾는다.
  //    (실제로는 path의 첫 점이 가장 가까울 가능성이 높지만,
  //     안전하게 전체를 검사한다.)
  //
  // 2) 그 점부터 path를 따라가며 누적 길이가 lookahead 이상 되는 지점을 목표점으로 사용
  //
  // 3) 끝까지 가도 없으면 마지막 점 사용
  bool compute_target_relative(double &tx, double &ty, double &Ld_used)
  {
    if (!path_fresh()) {
      return false;
    }

    const auto &poses = latest_path_.poses;
    if (poses.size() < 2) {
      return false;
    }

    // ---------------------------------------------
    // 1) 원점(차량 위치)에서 가장 가까운 점 찾기
    // ---------------------------------------------
    size_t nearest_i = 0;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < poses.size(); ++i) {
      const double px = poses[i].pose.position.x;
      const double py = poses[i].pose.position.y;

      const double d2 = px * px + py * py;  // 원점 기준 거리 제곱
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest_i = i;
      }
    }

    // ---------------------------------------------
    // 2) nearest_i부터 path를 따라가며
    //    누적 길이가 lookahead 이상 되는 점 찾기
    // ---------------------------------------------
    double acc = 0.0;

    for (size_t i = nearest_i; i + 1 < poses.size(); ++i) {
      const double x0 = poses[i].pose.position.x;
      const double y0 = poses[i].pose.position.y;
      const double x1 = poses[i + 1].pose.position.x;
      const double y1 = poses[i + 1].pose.position.y;

      acc += norm2d(x1 - x0, y1 - y0);

      if (acc >= Ld_default_) {
        tx = x1;
        ty = y1;
        Ld_used = norm2d(tx, ty);  // 실제 차량 원점에서 목표점까지의 직선거리
        return true;
      }
    }

    // ---------------------------------------------
    // 3) lookahead를 만족하는 점이 없으면 마지막 점 사용
    // ---------------------------------------------
    tx = poses.back().pose.position.x;
    ty = poses.back().pose.position.y;
    Ld_used = norm2d(tx, ty);
    return true;
  }

  // =============================================
  // 주기적으로 호출되는 메인 제어 루프
  // =============================================
  void on_timer()
  {
    // path가 없거나 오래됐으면 정지
    if (!path_fresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Path is missing or stale. Stop."
      );
      publish_stop();
      return;
    }

    // 목표점 계산
    double tx = 0.0;
    double ty = 0.0;
    double Ld_used = 0.0;

    if (!compute_target_relative(tx, ty, Ld_used)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Failed to compute target. Stop."
      );
      publish_stop();
      return;
    }

    // ---------------------------------------------
    // 안전 조건 1:
    // 목표점이 너무 가까우면 steering이 튈 수 있으므로 정지
    // ---------------------------------------------
    if (Ld_used < 1e-3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Target distance too small. Stop."
      );
      publish_stop();
      return;
    }

    // ---------------------------------------------
    // 안전 조건 2:
    // 목표점이 차량 뒤쪽에 있거나 너무 가까운 앞이면 정지
    // base_link 기준에서 x>0 이 앞쪽
    // ---------------------------------------------
    if (tx <= min_x_target_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[PP Relative] Target is behind or too close: tx=%.3f. Stop.",
        tx
      );
      publish_stop();
      return;
    }

    // =============================================
    // Pure Pursuit 조향 계산
    // =============================================
    //
    // relative path니까 이미
    //   x = 차량 기준 앞 방향
    //   y = 차량 기준 좌/우 방향
    //
    // 따라서 yaw 없이 바로 계산 가능
    //
    // kappa = 2 * y / Ld^2
    // delta = atan(L * kappa)
    //
    const double x_v = tx;
    const double y_v = ty;

    double kappa = (2.0 * y_v) / (Ld_used * Ld_used);
    double delta = std::atan(L_ * kappa);

    // 최대 조향각 제한
    delta = std::clamp(delta, -delta_max_, delta_max_);

    // =============================================
    // 제어 명령 발행
    // =============================================
    erp42_msgs::msg::ControlCommand cmd;
    cmd.speed = v_;
    cmd.steering = delta;
    cmd.brake = static_cast<uint8_t>(std::clamp(brake_run_, 0, 150));

    cmd_pub_->publish(cmd);

    // 디버깅 로그
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "[PP Relative] target=(%.2f, %.2f), Ld=%.2f, kappa=%.3f, delta=%.3f",
      tx, ty, Ld_used, kappa, delta
    );
  }

private:
  // 토픽 이름
  std::string path_topic_;
  std::string cmd_topic_;

  // 파라미터
  double L_{0.74};             // wheelbase
  double Ld_default_{1.2};     // 기본 lookahead 거리
  double v_{0.3};              // 목표 속도
  double delta_max_{0.314};    // 최대 조향각
  int brake_stop_{30};         // 정지 브레이크
  int brake_run_{0};           // 주행 브레이크
  double path_timeout_sec_{0.5};
  double min_x_target_{0.05};

  // ROS2 통신 객체
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<erp42_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 최신 path 저장
  nav_msgs::msg::Path latest_path_;
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitRelativeNode>());
  rclcpp::shutdown();
  return 0;
}