#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class LocalizationFuser : public rclcpp::Node{
public:
  LocalizationFuser()
  : Node("localization_fuser")
  {
    // ----------------------------
    // Parameters (NO hardcoding)
    // ----------------------------
    declare_parameter<std::string>("gps_pose_topic", "/gps_pose");
    declare_parameter<std::string>("odom_topic", "/erp42/odometry_wheel");
    declare_parameter<std::string>("out_pose_topic", "/local_pose");

    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("gps_timeout_s", 0.7);
    declare_parameter<bool>("use_gps_orientation", true); // GPS pose orientation을 그대로 쓸지

    gps_pose_topic_ = get_parameter("gps_pose_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    out_pose_topic_ = get_parameter("out_pose_topic").as_string();

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    gps_timeout_s_ = get_parameter("gps_timeout_s").as_double();
    use_gps_orientation_ = get_parameter("use_gps_orientation").as_bool();

    // ----------------------------
    // ROS I/O
    // ----------------------------
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      gps_pose_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationFuser::gpsCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationFuser::odomCallback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(out_pose_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // GPS timeout watchdog
    last_gps_time_ = now();
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationFuser::watchdogTick, this));

    RCLCPP_INFO(get_logger(),
      "localization_fuser started.\n"
      "  gps_pose_topic=%s\n"
      "  odom_topic=%s\n"
      "  out_pose_topic=%s\n"
      "  frames: %s -> %s\n"
      "  gps_timeout_s=%.2f, use_gps_orientation=%s",
      gps_pose_topic_.c_str(), odom_topic_.c_str(), out_pose_topic_.c_str(),
      map_frame_.c_str(), base_frame_.c_str(),
      gps_timeout_s_, use_gps_orientation_ ? "true" : "false");
  }

private:
  // -------- params --------
  std::string gps_pose_topic_;
  std::string odom_topic_;
  std::string out_pose_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double gps_timeout_s_{0.7};
  bool use_gps_orientation_{true};

  // -------- state --------
  bool gps_valid_{false};
  bool initialized_{false};

  double last_odom_x_{0.0};
  double last_odom_y_{0.0};

  rclcpp::Time last_gps_time_{0,0,RCL_ROS_TIME};

  geometry_msgs::msg::PoseStamped current_pose_;

  // -------- ROS --------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  void gpsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    last_gps_time_ = now();

    // GPS가 들어오면 GPS pose로 재동기화
    current_pose_ = *msg;

    // 필요하면 orientation은 유지/무시 선택 가능
    if (!use_gps_orientation_) {
      // orientation을 쓰지 않는다면 기존 orientation 유지 (없으면 identity)
      // (원하면 yaw 추정 로직을 별도에서 넣는 게 더 깔끔)
      // 여기서는 "그대로 둔다" 전략
    }

    gps_valid_ = true;
    initialized_ = true;

    publishPose();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (!initialized_) {
      last_odom_x_ = x;
      last_odom_y_ = y;
      return;
    }

    const double dx = x - last_odom_x_;
    const double dy = y - last_odom_y_;

    last_odom_x_ = x;
    last_odom_y_ = y;

    // GPS가 끊겼을 때만 wheel odom을 적분
    if (!gps_valid_) {
      current_pose_.pose.position.x += dx;
      current_pose_.pose.position.y += dy;
    }

    publishPose();
  }

  void watchdogTick(){
    if (!initialized_) return;

    const double dt = (now() - last_gps_time_).seconds();
    if (dt > gps_timeout_s_) {
      if (gps_valid_) {
        gps_valid_ = false;
        RCLCPP_WARN(get_logger(), "GPS timeout: dt=%.2fs > %.2fs. Switch to ODOM integration.",
                    dt, gps_timeout_s_);
      }
    }
  }

  void publishPose(){
    current_pose_.header.stamp = now();
    current_pose_.header.frame_id = map_frame_;

    pose_pub_->publish(current_pose_);

    // TF: map -> base_link (frame명도 파라미터)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_pose_.header.stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = base_frame_;

    t.transform.translation.x = current_pose_.pose.position.x;
    t.transform.translation.y = current_pose_.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = current_pose_.pose.orientation; // yaw 포함하면 더 좋음
    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationFuser>());
  rclcpp::shutdown();
  return 0;
}