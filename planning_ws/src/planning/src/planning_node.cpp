#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "planning/hybrid_astar.hpp"

using planning::HybridAStar;
using planning::State;
using planning::Grid;

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode()
  : Node("planning_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    planner_(0.32,   // wheelbase (임시)
             0.45,   // delta_max rad (임시)
             0.20,   // step (m)
             72,     // theta bins
             0.10)   // grid resolution (m)
  {
    // params
    this->declare_parameter<std::string>("fixed_frame", "map");
    this->declare_parameter<std::string>("sensor_frame", "base_link");
    this->declare_parameter<std::string>("cones_topic", "/perception/cones");
    this->declare_parameter<double>("grid_resolution", 0.10);
    this->declare_parameter<double>("grid_size_m", 10.0);      // 10m x 10m
    this->declare_parameter<double>("inflation_m", 0.35);      // 장애물 부풀리기
    this->declare_parameter<double>("lookahead_m", 6.0);       // goal ahead
    this->declare_parameter<double>("robot_x", 1.0);           // 테스트 ego 위치(나중에 odom/pose로 대체)
    this->declare_parameter<double>("robot_y", 1.0);
    this->declare_parameter<double>("robot_yaw", 0.0);

    fixed_frame_   = this->get_parameter("fixed_frame").as_string();
    sensor_frame_  = this->get_parameter("sensor_frame").as_string();
    cones_topic_   = this->get_parameter("cones_topic").as_string();
    res_           = this->get_parameter("grid_resolution").as_double();
    grid_size_m_   = this->get_parameter("grid_size_m").as_double();
    inflation_m_   = this->get_parameter("inflation_m").as_double();
    lookahead_m_   = this->get_parameter("lookahead_m").as_double();

    W_ = static_cast<int>(std::round(grid_size_m_ / res_));
    H_ = static_cast<int>(std::round(grid_size_m_ / res_));

    // pubs
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 1);
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/planning/grid", 1);

    // sub (cones)
    cones_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cones_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlanningNode::onCones, this, std::placeholders::_1));

    // timer: plan at 5~10Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&PlanningNode::planOnce, this));

    // init grid
    grid_.assign(W_, std::vector<int>(H_, 0));
  }

private:
  // ----- grid utils -----
  inline bool worldToGridLocal(double x, double y, int &ix, int &iy) const
  {
    // 로컬 costmap: (0,0)이 좌하단이 아니라 "origin = robot center - size/2" 방식으로 잡을거야
    // 여기서는 map 좌표를 "로컬 맵 좌표"로 변환한 다음 grid index로 바꾼다고 생각하면 됨.
    // local frame origin(미터) : (origin_x_, origin_y_)
    ix = static_cast<int>(std::floor((x - origin_x_) / res_));
    iy = static_cast<int>(std::floor((y - origin_y_) / res_));
    if (ix < 0 || iy < 0 || ix >= W_ || iy >= H_) return false;
    return true;
  }

  void clearGrid()
  {
    for (int x = 0; x < W_; ++x)
      std::fill(grid_[x].begin(), grid_[x].end(), 0);
  }

  void stampObstacleCell(int cx, int cy)
  {
    if (cx < 0 || cy < 0 || cx >= W_ || cy >= H_) return;
    grid_[cx][cy] = 1;
  }

  void stampInflatedObstacle(double wx, double wy)
  {
    // wx, wy: map frame
    int cx, cy;
    if (!worldToGridLocal(wx, wy, cx, cy)) return;

    const int R = std::max(1, static_cast<int>(std::round(inflation_m_ / res_)));

    for (int dx = -R; dx <= R; ++dx)
    {
      for (int dy = -R; dy <= R; ++dy)
      {
        int nx = cx + dx;
        int ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= W_ || ny >= H_) continue;
        if ((dx*dx + dy*dy) <= (R*R)) grid_[nx][ny] = 1;
      }
    }
  }

  void publishGrid()
  {
    nav_msgs::msg::OccupancyGrid og;
    og.header.frame_id = fixed_frame_;
    og.header.stamp = now();

    og.info.resolution = res_;
    og.info.width = W_;
    og.info.height = H_;

    // origin: local map의 (0,0) world 좌표
    og.info.origin.position.x = origin_x_;
    og.info.origin.position.y = origin_y_;
    og.info.origin.orientation.w = 1.0;

    og.data.resize(W_ * H_, 0);

    // grid_[x][y] -> data[y*W + x]
    for (int x = 0; x < W_; ++x)
      for (int y = 0; y < H_; ++y)
        og.data[y * W_ + x] = (grid_[x][y] == 1) ? 100 : 0;

    grid_pub_->publish(og);
  }

  // ----- cones callback -----
  void onCones(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // frame이 이미 map이면 그대로 사용
  if (msg->header.frame_id == fixed_frame_)
  {
    latest_cones_ = *msg;
    has_cones_ = true;
    return;
  }

  // 아니면 TF로 map으로 변환
  sensor_msgs::msg::PointCloud2 cloud_map;
  try
  {
    auto tf = tf_buffer_.lookupTransform(
      fixed_frame_,               // target
      msg->header.frame_id,        // source
      tf2::TimePointZero);

    tf2::doTransform(*msg, cloud_map, tf);

    latest_cones_ = cloud_map;
    has_cones_ = true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF transform failed (%s -> %s): %s",
                         msg->header.frame_id.c_str(), fixed_frame_.c_str(), e.what());
    has_cones_ = false;
  }
}

  // ----- planning cycle -----
  void planOnce()
  {
    // 0) ego pose (지금은 파라미터로 테스트, 나중에 odom/pose subscribe로 바꿀 것)
    double rx = this->get_parameter("robot_x").as_double();
    double ry = this->get_parameter("robot_y").as_double();
    double ryaw = this->get_parameter("robot_yaw").as_double();

    // local map origin = robot 중심 기준 size/2 만큼 빼기 (map frame)
    origin_x_ = rx - grid_size_m_ * 0.5;
    origin_y_ = ry - grid_size_m_ * 0.5;

    // 1) grid clear
    clearGrid();

    // 2) cones -> obstacles
    if (has_cones_)
    {
      // PointCloud2 iterate: x,y,z float32
      // 단순하게 sensor_msgs::PointCloud2Iterator 써도 되는데,
      // 여기서는 최소 의존성으로 "raw fields" iterator 방식 사용 권장.
      // (iterator 쓰고 싶으면 내가 다음 메시지에서 더 깔끔하게 바꿔줄게.)
      // ---- 간단 버전: PointCloud2Iterator 사용 ----
      sensor_msgs::PointCloud2ConstIterator<float> it_x(latest_cones_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(latest_cones_, "y");

      for (; it_x != it_x.end(); ++it_x, ++it_y)
      {
        double x = static_cast<double>(*it_x);
        double y = static_cast<double>(*it_y);

        // 로컬 10m 안쪽만 반영(밖이면 grid 변환에서 걸러짐)
        stampInflatedObstacle(x, y);
      }
    }

    // 3) goal = lookahead 만큼 전방(현재 yaw 기준)
    State start{rx, ry, ryaw};
    State goal{
      rx + lookahead_m_ * std::cos(ryaw),
      ry + lookahead_m_ * std::sin(ryaw),
      ryaw
    };

    // 4) plan
    auto path_states = planner_.plan(start, goal, grid_);

    // 5) publish grid + path
    publishGrid();
    publishPath(path_states);
  }

  void publishPath(const std::vector<State>& path_states)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = fixed_frame_;
    msg.header.stamp = now();

    for (const auto& s : path_states)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header = msg.header;
      p.pose.position.x = s.x;
      p.pose.position.y = s.y;
      p.pose.orientation = yawToQuat(s.theta);
      msg.poses.push_back(p);
    }

    path_pub_->publish(msg);
  }

private:
  // pubs/subs
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // planner
  HybridAStar planner_;

  // params/state
  std::string fixed_frame_;
  std::string sensor_frame_;
  std::string cones_topic_;

  double res_{0.10};
  double grid_size_m_{10.0};
  double inflation_m_{0.35};
  double lookahead_m_{6.0};

  int W_{100};
  int H_{100};

  double origin_x_{0.0};
  double origin_y_{0.0};

  Grid grid_;

  // latest cones
  sensor_msgs::msg::PointCloud2 latest_cones_;
  bool has_cones_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}