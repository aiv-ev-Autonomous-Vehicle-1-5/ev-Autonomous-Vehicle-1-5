#ifndef LIDAR_TRACKING_BYTE_TRACKER_HPP_
#define LIDAR_TRACKING_BYTE_TRACKER_HPP_

#include "lidar_tracking/kalman_filter.hpp"
#include <lidar_interfaces/msg/cone.hpp>
#include <lidar_interfaces/msg/tracked_cone.hpp>
#include <vector>
#include <memory>

namespace lidar_tracking
{

struct Track {
  uint32_t id;
  int32_t label;
  Eigen::Vector3f dimensions;
  float confidence;
  KalmanFilter kf;
  float z;

  int time_since_update;
  int hits;

  enum State { Tracked, Lost, Removed };
  State state;
};

class ByteTracker
{
public:
  ByteTracker(float high_thresh, float low_thresh,
              float match_dist_high, float match_dist_low,
              int max_time_lost, float new_track_thresh,
              float sigma_a, float sigma_r);

  std::vector<lidar_interfaces::msg::TrackedCone> update(
    const std::vector<lidar_interfaces::msg::Cone> & detections,
    float dt);

private:
  float high_thresh_;
  float low_thresh_;
  float match_dist_high_;
  float match_dist_low_;
  int max_time_lost_;
  float new_track_thresh_;
  float sigma_a_;
  float sigma_r_;

  uint32_t next_id_;
  std::vector<std::shared_ptr<Track>> tracks_;
  std::vector<std::shared_ptr<Track>> lost_tracks_;

  float computeDistance(const std::shared_ptr<Track> & track,
                        const lidar_interfaces::msg::Cone & det);

  void associate(
    const std::vector<std::shared_ptr<Track>> & tracks,
    const std::vector<const lidar_interfaces::msg::Cone *> & detections,
    float distance_threshold,
    std::vector<std::pair<int, int>> & matches,
    std::vector<int> & unmatched_tracks,
    std::vector<int> & unmatched_detections);
};

} // namespace lidar_tracking

#endif // LIDAR_TRACKING_BYTE_TRACKER_HPP_
