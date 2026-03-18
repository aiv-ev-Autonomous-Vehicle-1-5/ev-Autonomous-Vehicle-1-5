#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class LocalizationFuser : public rclcpp::Node{
public:
    LocalizationFuser()
    : Node("localization_fuser"),
      gps_valid_(false),
      last_odom_x_(0.0),
      last_odom_y_(0.0),
      initialized_(false){
        gps_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gps_pose", 10,
            std::bind(&LocalizationFuser::gpsCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/erp42/odometry_wheel", 10,
            std::bind(&LocalizationFuser::odomCallback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/local_pose", 10);

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "Localization fusion node started");
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::PoseStamped current_pose_;

    bool gps_valid_;
    bool initialized_;

    double last_odom_x_;
    double last_odom_y_;

    void gpsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        current_pose_ = *msg;
        gps_valid_ = true;
        initialized_ = true;

        publishPose();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (!initialized_){
            last_odom_x_ = x;
            last_odom_y_ = y;
            return;
        }

        double dx = x - last_odom_x_;
        double dy = y - last_odom_y_;

        last_odom_x_ = x;
        last_odom_y_ = y;

        if (!gps_valid_){
            current_pose_.pose.position.x += dx;
            current_pose_.pose.position.y += dy;
        }

        publishPose();
    }

    void publishPose(){
        current_pose_.header.stamp = now();
        current_pose_.header.frame_id = "map";

        pose_pub_->publish(current_pose_);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = current_pose_.pose.position.x;
        t.transform.translation.y = current_pose_.pose.position.y;
        t.transform.translation.z = 0.0;

        t.transform.rotation = current_pose_.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationFuser>());
    rclcpp::shutdown();
    return 0;
}