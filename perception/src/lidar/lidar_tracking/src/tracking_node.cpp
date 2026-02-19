#include "lidar_tracking/tracking_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_tracking
{

TrackingNode::TrackingNode(const rclcpp::NodeOptions & options)
: Node("tracking_node", options)
{
  // Parameters
  float high_thresh = this->declare_parameter("bytetrack.high_thresh", 0.6f);
  float low_thresh = this->declare_parameter("bytetrack.low_thresh", 0.1f);
  float match_dist_high = this->declare_parameter("bytetrack.match_distance_high", 0.5f);
  float match_dist_low = this->declare_parameter("bytetrack.match_distance_low", 0.8f);
  int max_time_lost = this->declare_parameter("bytetrack.max_time_lost", 30);
  float new_track_thresh = this->declare_parameter("bytetrack.new_track_thresh", 0.7f);
  float sigma_a = this->declare_parameter("kalman_filter.process_noise_cov", 1.0f);
  float sigma_r = this->declare_parameter("kalman_filter.measure_noise_cov", 0.1f);
  dt_default_ = this->declare_parameter("kalman_filter.dt", 0.1f);

  tracker_ = std::make_unique<ByteTracker>(
    high_thresh, low_thresh, match_dist_high, match_dist_low,
    max_time_lost, new_track_thresh, sigma_a, sigma_r);

  // QoS
  auto qos = rclcpp::SensorDataQoS();
  
  sub_ = this->create_subscription<lidar_interfaces::msg::ConeArray>(
    "input", qos,
    std::bind(&TrackingNode::callback, this, std::placeholders::_1));
    
  pub_ = this->create_publisher<lidar_interfaces::msg::TrackedConeArray>("output", 10); // 다음 단계 msg
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/tracking_markers", 10); //rendering 용

  RCLCPP_INFO(this->get_logger(), "TrackingNode initialized with HighThresh: %.2f, LowThresh: %.2f", 
    high_thresh, low_thresh);
}

void TrackingNode::callback(const lidar_interfaces::msg::ConeArray::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  float dt = dt_default_;

  if (initialized_) {
    double seconds = (current_time - last_time_).seconds();
    if (seconds > 0.001) {
      dt = static_cast<float>(seconds);
    }
  } else {
    initialized_ = true;
  }
  last_time_ = current_time;

  // Run Tracker
  std::vector<lidar_interfaces::msg::TrackedCone> tracked_cones = 
    tracker_->update(msg->cones, dt);

  // Publish Output
  lidar_interfaces::msg::TrackedConeArray out_msg;
  out_msg.header = msg->header;
  out_msg.cones = tracked_cones;
  pub_->publish(out_msg);

  // Publish Visualization Markers
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);

  for (const auto& cone : tracked_cones) {
    visualization_msgs::msg::Marker m;
    m.header = msg->header;
    m.ns = "tracked_cone";
    m.id = cone.track_id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = cone.position;
    m.pose.orientation.w = 1.0;
    m.scale.x = cone.dimensions.x;
    m.scale.y = cone.dimensions.y;
    m.scale.z = cone.dimensions.z;
    
    // Bright color palette by track ID
    static const float palette[][3] = {
      {1.0f, 0.27f, 0.27f}, {0.27f, 1.0f, 0.27f}, {0.27f, 0.67f, 1.0f},
      {1.0f, 1.0f, 0.27f},  {1.0f, 0.27f, 1.0f},  {0.27f, 1.0f, 1.0f},
      {1.0f, 0.67f, 0.27f}, {0.67f, 0.53f, 1.0f},  {0.53f, 1.0f, 0.67f},
      {1.0f, 0.53f, 0.8f},  {0.8f, 1.0f, 0.27f},  {0.27f, 0.8f, 1.0f},
    };
    static const int palette_size = sizeof(palette) / sizeof(palette[0]);
    int ci = cone.track_id % palette_size;
    m.color.r = palette[ci][0];
    m.color.g = palette[ci][1];
    m.color.b = palette[ci][2];
    m.color.a = 0.8f;
    
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(m);

    // Text ID
    visualization_msgs::msg::Marker text;
    text.header = msg->header;
    text.ns = "tracked_id";
    text.id = cone.track_id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position = cone.position;
    text.pose.position.z += cone.dimensions.z + 0.2;
    text.scale.z = 0.3;
    text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
    text.text = "ID: " + std::to_string(cone.track_id);
    text.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(text);
    
    // Velocity Arrow
    if (std::abs(cone.velocity.x) > 0.1 || std::abs(cone.velocity.y) > 0.1) {
      visualization_msgs::msg::Marker arrow;
      arrow.header = msg->header;
      arrow.ns = "velocity";
      arrow.id = cone.track_id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.points.push_back(cone.position);
      
      geometry_msgs::msg::Point end_pt;
      end_pt.x = cone.position.x + cone.velocity.x;
      end_pt.y = cone.position.y + cone.velocity.y;
      end_pt.z = cone.position.z;
      arrow.points.push_back(end_pt);
      
      arrow.scale.x = 0.05; // shaft diameter
      arrow.scale.y = 0.1;  // head diameter
      arrow.scale.z = 0.1;  // head length
      arrow.color.r = 1.0; arrow.color.a = 0.8;
      arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(arrow);
    }
  }
  
  marker_pub_->publish(marker_array);
}

} // namespace lidar_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_tracking::TrackingNode)
