#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "dbscan_clustering/dbscan_gpu.cuh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

namespace dbscan_clustering {

class DBSCANNode : public rclcpp::Node {
public:
  explicit DBSCANNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dbscan_clustering", options)
  {
    declare_parameter<std::string>("input_topic", "/patchworkpp/nonground");
    declare_parameter<std::string>("output_topic", "/pointcloud/clustered");
    declare_parameter<double>("eps", 0.35);
    declare_parameter<int>("min_points", 12);
    declare_parameter<int>("max_neighbors", 256);
    declare_parameter<int>("max_points", 200000);
    declare_parameter<int>("queue_size", 1);

    declare_parameter<double>("roi_min_x", 0.5);
    declare_parameter<double>("roi_max_x", 20.0);
    declare_parameter<double>("roi_min_y", -5.0);
    declare_parameter<double>("roi_max_y", 5.0);
    declare_parameter<double>("roi_min_z", -2.0);
    declare_parameter<double>("roi_max_z", 2.0);

    declare_parameter<bool>("enable_azimuth_ground_suppression", true);
    declare_parameter<double>("azimuth_ground_min_range", 2.0);
    declare_parameter<double>("azimuth_ground_z_threshold", 0.12);
    declare_parameter<int>("azimuth_ground_bins", 720);
    declare_parameter<bool>("cluster_xy_only", true);
    declare_parameter<double>("z_weight", 1.0);
    declare_parameter<double>("z_distance_scale", 0.0);

    const auto in_topic = get_parameter("input_topic").as_string();
    const auto out_topic = get_parameter("output_topic").as_string();
    eps_ = get_parameter("eps").as_double();
    min_points_ = get_parameter("min_points").as_int();
    max_neighbors_ = get_parameter("max_neighbors").as_int();
    max_points_ = get_parameter("max_points").as_int();
    const int queue_size = get_parameter("queue_size").as_int();

    roi_min_x_ = get_parameter("roi_min_x").as_double();
    roi_max_x_ = get_parameter("roi_max_x").as_double();
    roi_min_y_ = get_parameter("roi_min_y").as_double();
    roi_max_y_ = get_parameter("roi_max_y").as_double();
    roi_min_z_ = get_parameter("roi_min_z").as_double();
    roi_max_z_ = get_parameter("roi_max_z").as_double();

    enable_azimuth_ground_suppression_ = get_parameter("enable_azimuth_ground_suppression").as_bool();
    azimuth_ground_min_range_ = get_parameter("azimuth_ground_min_range").as_double();
    azimuth_ground_z_threshold_ = get_parameter("azimuth_ground_z_threshold").as_double();
    azimuth_ground_bins_ = std::max(8, static_cast<int>(get_parameter("azimuth_ground_bins").as_int()));
    cluster_xy_only_ = get_parameter("cluster_xy_only").as_bool();
    z_weight_ = get_parameter("z_weight").as_double();
    z_distance_scale_ = get_parameter("z_distance_scale").as_double();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&DBSCANNode::cloudCallback, this, _1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "dbscan_clustering started in:%s out:%s eps:%.3f min_points:%d ROI x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f] az_ground:%s xy_only:%s",
      in_topic.c_str(), out_topic.c_str(), eps_, min_points_,
      roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_,
      enable_azimuth_ground_suppression_ ? "on" : "off",
      cluster_xy_only_ ? "on" : "off");
  }

private:
  struct CandidatePoint
  {
    float x;
    float y;
    float z;
  };

  struct FieldMeta
  {
    int offset{-1};
    uint8_t datatype{0};
  };

  static bool read_float_field(
    const uint8_t * point_ptr,
    const FieldMeta & meta,
    float & out)
  {
    if (meta.offset < 0) {
      return false;
    }
    if (meta.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      std::memcpy(&out, point_ptr + meta.offset, sizeof(float));
      return true;
    }
    if (meta.datatype == sensor_msgs::msg::PointField::FLOAT64) {
      double tmp = 0.0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(double));
      out = static_cast<float>(tmp);
      return true;
    }
    return false;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    int n_points = static_cast<int>(msg->width * msg->height);
    if (n_points <= 0) {
      return;
    }
    if (n_points > max_points_) {
      n_points = max_points_;
    }

    FieldMeta meta_x;
    FieldMeta meta_y;
    FieldMeta meta_z;

    for (const auto & f : msg->fields) {
      if (f.name == "x") {
        meta_x.offset = static_cast<int>(f.offset);
        meta_x.datatype = f.datatype;
      } else if (f.name == "y") {
        meta_y.offset = static_cast<int>(f.offset);
        meta_y.datatype = f.datatype;
      } else if (f.name == "z") {
        meta_z.offset = static_cast<int>(f.offset);
        meta_z.datatype = f.datatype;
      }
    }

    if (meta_x.offset < 0 || meta_y.offset < 0 || meta_z.offset < 0) {
      RCLCPP_ERROR(get_logger(), "PointCloud2 missing x/y/z fields");
      return;
    }

    const size_t point_step = static_cast<size_t>(msg->point_step);
    if (point_step == 0 || msg->data.size() < static_cast<size_t>(n_points) * point_step) {
      RCLCPP_WARN(get_logger(), "Invalid PointCloud2 layout");
      return;
    }

    std::vector<CandidatePoint> roi_points;
    roi_points.reserve(static_cast<size_t>(n_points));

    for (int i = 0; i < n_points; ++i) {
      const uint8_t * point_ptr = &msg->data[static_cast<size_t>(i) * point_step];

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!read_float_field(point_ptr, meta_x, x) ||
        !read_float_field(point_ptr, meta_y, y) ||
        !read_float_field(point_ptr, meta_z, z))
      {
        continue;
      }

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (x < roi_min_x_ || x > roi_max_x_ ||
        y < roi_min_y_ || y > roi_max_y_ ||
        z < roi_min_z_ || z > roi_max_z_)
      {
        continue;
      }

      roi_points.push_back({x, y, z});
    }

    if (roi_points.empty()) {
      return;
    }

    std::vector<CandidatePoint> nonground_points = roi_points;

    if (enable_azimuth_ground_suppression_ && !nonground_points.empty()) {
      const size_t bins = static_cast<size_t>(azimuth_ground_bins_);
      std::vector<float> min_z_by_bin(bins, std::numeric_limits<float>::infinity());
      const float two_pi = 2.0F * static_cast<float>(M_PI);

      for (const auto & p : nonground_points) {
        const float range_xy = std::hypot(p.x, p.y);
        if (range_xy < static_cast<float>(azimuth_ground_min_range_)) {
          continue;
        }
        float az = std::atan2(p.y, p.x);
        if (az < 0.0F) {
          az += two_pi;
        }
        int bin = static_cast<int>((az / two_pi) * static_cast<float>(bins));
        bin = std::clamp(bin, 0, static_cast<int>(bins) - 1);
        min_z_by_bin[static_cast<size_t>(bin)] =
          std::min(min_z_by_bin[static_cast<size_t>(bin)], p.z);
      }

      std::vector<CandidatePoint> filtered;
      filtered.reserve(nonground_points.size());
      for (const auto & p : nonground_points) {
        const float range_xy = std::hypot(p.x, p.y);
        if (range_xy < static_cast<float>(azimuth_ground_min_range_)) {
          filtered.push_back(p);
          continue;
        }
        float az = std::atan2(p.y, p.x);
        if (az < 0.0F) {
          az += two_pi;
        }
        int bin = static_cast<int>((az / two_pi) * static_cast<float>(bins));
        bin = std::clamp(bin, 0, static_cast<int>(bins) - 1);
        const float z_ref = min_z_by_bin[static_cast<size_t>(bin)];
        if (!std::isfinite(z_ref) ||
          p.z > z_ref + static_cast<float>(azimuth_ground_z_threshold_))
        {
          filtered.push_back(p);
        }
      }
      nonground_points.swap(filtered);
    }

    const int m = static_cast<int>(nonground_points.size());
    if (m == 0) {
      return;
    }

    const int UNVISITED = -2;
    std::vector<int32_t> labels(static_cast<size_t>(m), UNVISITED);
    int cluster_id = 0;

    std::vector<float> xyz(static_cast<size_t>(m) * 3U);
    for (int i = 0; i < m; ++i) {
      const auto & pt = nonground_points[static_cast<size_t>(i)];
      xyz[static_cast<size_t>(3 * i + 0)] = pt.x;
      xyz[static_cast<size_t>(3 * i + 1)] = pt.y;
      if (cluster_xy_only_) {
        xyz[static_cast<size_t>(3 * i + 2)] = 0.0F;
      } else {
        const float range_xy = std::hypot(pt.x, pt.y);
        xyz[static_cast<size_t>(3 * i + 2)] = pt.z * static_cast<float>(z_weight_)
          / (1.0F + range_xy * static_cast<float>(z_distance_scale_));
      }
    }
    std::vector<int> neighbors(static_cast<size_t>(m) * static_cast<size_t>(max_neighbors_), -1);
    std::vector<int> neighbor_counts(static_cast<size_t>(m), 0);
    dbscan_gpu_query_neighbors(
      xyz.data(), m, static_cast<float>(eps_), max_neighbors_,
      neighbors.data(), neighbor_counts.data());

    int saturated = 0;
    for (int i = 0; i < m; ++i) {
      if (neighbor_counts[static_cast<size_t>(i)] >= max_neighbors_) {
        ++saturated;
      }
    }
    if (saturated > 0) {
      const double sat_ratio = static_cast<double>(saturated) / static_cast<double>(m);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "neighbor cap saturated: %d/%d (%.1f%%). Increase max_neighbors or lower eps.",
        saturated, m, sat_ratio * 100.0);
    }

    for (int i = 0; i < m; ++i) {
      if (labels[static_cast<size_t>(i)] != UNVISITED) {
        continue;
      }

      if (neighbor_counts[static_cast<size_t>(i)] < min_points_) {
        labels[static_cast<size_t>(i)] = -1;
        continue;
      }

      std::queue<int> q;
      labels[static_cast<size_t>(i)] = cluster_id;
      q.push(i);

      while (!q.empty()) {
        const int cur = q.front();
        q.pop();

        const int cur_count = neighbor_counts[static_cast<size_t>(cur)];
        if (cur_count < min_points_) {
          continue;
        }

        const size_t row_begin = static_cast<size_t>(cur) * static_cast<size_t>(max_neighbors_);
        for (int k = 0; k < cur_count; ++k) {
          const int nb = neighbors[row_begin + static_cast<size_t>(k)];
          if (nb < 0 || nb >= m) {
            continue;
          }
          int32_t & lbl = labels[static_cast<size_t>(nb)];
          if (lbl == UNVISITED) {
            lbl = cluster_id;
            q.push(nb);
          } else if (lbl == -1) {
            lbl = cluster_id;
          }
        }
      }

      ++cluster_id;
    }

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = msg->header;
    out->height = 1;
    out->width = static_cast<uint32_t>(m);
    out->is_bigendian = msg->is_bigendian;
    out->is_dense = false;

    sensor_msgs::msg::PointField fx;
    fx.name = "x";
    fx.offset = 0;
    fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fx.count = 1;

    sensor_msgs::msg::PointField fy;
    fy.name = "y";
    fy.offset = 4;
    fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fy.count = 1;

    sensor_msgs::msg::PointField fz;
    fz.name = "z";
    fz.offset = 8;
    fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fz.count = 1;

    sensor_msgs::msg::PointField fcluster;
    fcluster.name = "cluster_id";
    fcluster.offset = 12;
    fcluster.datatype = sensor_msgs::msg::PointField::INT32;
    fcluster.count = 1;

    out->fields = {fx, fy, fz, fcluster};
    out->point_step = 16;
    out->row_step = out->point_step * out->width;
    out->data.resize(static_cast<size_t>(out->row_step));

    sensor_msgs::PointCloud2Iterator<float> out_x(*out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*out, "z");
    sensor_msgs::PointCloud2Iterator<int32_t> out_cluster(*out, "cluster_id");

    for (int i = 0; i < m; ++i) {
      const auto & p = nonground_points[static_cast<size_t>(i)];
      const int32_t lbl = labels[static_cast<size_t>(i)];

      *out_x = p.x; ++out_x;
      *out_y = p.y; ++out_y;
      *out_z = p.z; ++out_z;
      *out_cluster = lbl; ++out_cluster;
    }

    pub_->publish(std::move(out));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "points in:%d roi:%zu nonground:%d clusters:%d",
      static_cast<int>(msg->width * msg->height),
      roi_points.size(), m, cluster_id);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  double eps_{0.35};
  int min_points_{12};
  int max_neighbors_{256};
  int max_points_{200000};

  double roi_min_x_{0.5};
  double roi_max_x_{20.0};
  double roi_min_y_{-5.0};
  double roi_max_y_{5.0};
  double roi_min_z_{-2.0};
  double roi_max_z_{2.0};

  bool enable_azimuth_ground_suppression_{true};
  double azimuth_ground_min_range_{2.0};
  double azimuth_ground_z_threshold_{0.12};
  int azimuth_ground_bins_{720};
  bool cluster_xy_only_{true};
  double z_weight_{1.0};
  double z_distance_scale_{0.0};
};

}  // namespace dbscan_clustering

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dbscan_clustering::DBSCANNode)
