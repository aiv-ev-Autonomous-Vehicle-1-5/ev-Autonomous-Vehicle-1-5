#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class FakeConePublisher : public rclcpp::Node
{
public:
    FakeConePublisher()
    : Node("fake_cone_publisher")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/cones", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FakeConePublisher::publishCones, this));
    }

private:
    void publishCones()
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = this->now();
        cloud.height = 1;
        cloud.width = 2;  // 2 cones
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(2);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        // Cone 1
        iter_x[0] = 3.0;
        iter_y[0] = 0.0;
        iter_z[0] = 0.0;

        // Cone 2
        iter_x[1] = 4.0;
        iter_y[1] = 0.0;
        iter_z[1] = 0.0;

        pub_->publish(cloud);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeConePublisher>());
    rclcpp::shutdown();
    return 0;
}