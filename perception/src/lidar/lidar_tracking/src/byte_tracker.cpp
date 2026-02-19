#include "lidar_tracking/byte_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace lidar_tracking
{

ByteTracker::ByteTracker(float high_thresh, float low_thresh,
                         float match_dist_high, float match_dist_low,
                         int max_time_lost, float new_track_thresh,
                         float sigma_a, float sigma_r)
  : high_thresh_(high_thresh),
    low_thresh_(low_thresh),
    match_dist_high_(match_dist_high),
    match_dist_low_(match_dist_low),
    max_time_lost_(max_time_lost),
    new_track_thresh_(new_track_thresh),
    sigma_a_(sigma_a),
    sigma_r_(sigma_r),
    next_id_(0)
{
}

float ByteTracker::computeDistance(
  const std::shared_ptr<Track> & track,
  const lidar_interfaces::msg::Cone & det)
{
  Eigen::Vector4f s = track->kf.getState();
  float dx = s(0) - static_cast<float>(det.position.x);
  float dy = s(1) - static_cast<float>(det.position.y);
  return std::sqrt(dx * dx + dy * dy);
}

void ByteTracker::associate(
  const std::vector<std::shared_ptr<Track>> & tracks,
  const std::vector<const lidar_interfaces::msg::Cone *> & detections,
  float distance_threshold,
  std::vector<std::pair<int, int>> & matches,
  std::vector<int> & unmatched_tracks,
  std::vector<int> & unmatched_detections)
{
  matches.clear();
  unmatched_tracks.clear();
  unmatched_detections.clear();

  if (tracks.empty()) {
    for (size_t i = 0; i < detections.size(); ++i)
      unmatched_detections.push_back(static_cast<int>(i));
    return;
  }
  if (detections.empty()) {
    for (size_t i = 0; i < tracks.size(); ++i)
      unmatched_tracks.push_back(static_cast<int>(i));
    return;
  }

  // Build distance pairs under threshold, then greedy sort-match
  struct Pair { float dist; int t; int d; };
  std::vector<Pair> pairs;
  for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
    for (int j = 0; j < static_cast<int>(detections.size()); ++j) {
      float d = computeDistance(tracks[i], *detections[j]);
      if (d < distance_threshold)
        pairs.push_back({d, i, j});
    }
  }

  std::sort(pairs.begin(), pairs.end(),
    [](const Pair & a, const Pair & b) { return a.dist < b.dist; });

  std::vector<bool> t_used(tracks.size(), false);
  std::vector<bool> d_used(detections.size(), false);

  for (const auto & p : pairs) {
    if (!t_used[p.t] && !d_used[p.d]) {
      matches.push_back({p.t, p.d});
      t_used[p.t] = true;
      d_used[p.d] = true;
    }
  }

  for (int i = 0; i < static_cast<int>(tracks.size()); ++i)
    if (!t_used[i]) unmatched_tracks.push_back(i);
  for (int j = 0; j < static_cast<int>(detections.size()); ++j)
    if (!d_used[j]) unmatched_detections.push_back(j);
}

// Helper: update a track with a detection
static void updateTrack(std::shared_ptr<Track> & track,
                        const lidar_interfaces::msg::Cone & det)
{
  Eigen::Vector2f z(static_cast<float>(det.position.x),
                    static_cast<float>(det.position.y));
  track->kf.update(z);
  track->z = static_cast<float>(det.position.z);
  track->dimensions = Eigen::Vector3f(
    static_cast<float>(det.dimensions.x),
    static_cast<float>(det.dimensions.y),
    static_cast<float>(det.dimensions.z));
  track->confidence = det.confidence;
  track->label = det.label;
  track->time_since_update = 0;
  track->hits++;
  track->state = Track::Tracked;
}

std::vector<lidar_interfaces::msg::TrackedCone> ByteTracker::update(
  const std::vector<lidar_interfaces::msg::Cone> & detections,
  float dt)
{
  // === Split detections by confidence ===
  std::vector<const lidar_interfaces::msg::Cone *> d_high, d_low;
  for (const auto & det : detections) {
    if (det.confidence >= high_thresh_)
      d_high.push_back(&det);
    else if (det.confidence > low_thresh_)
      d_low.push_back(&det);
  }

  // === Predict all tracks ===
  for (auto & t : tracks_)      t->kf.predict(dt);
  for (auto & t : lost_tracks_) t->kf.predict(dt);

  // === Stage 1: Tracked tracks vs D_high ===
  std::vector<std::pair<int, int>> matches1;
  std::vector<int> unmatched_tracked_idx, unmatched_dhigh_idx;
  associate(tracks_, d_high, match_dist_high_,
            matches1, unmatched_tracked_idx, unmatched_dhigh_idx);

  std::vector<std::shared_ptr<Track>> output_tracked;

  for (const auto & m : matches1) {
    auto trk = tracks_[m.first];
    updateTrack(trk, *d_high[m.second]);
    output_tracked.push_back(trk);
  }

  // === Stage 2: Unmatched tracked vs D_low ===
  std::vector<std::shared_ptr<Track>> remain_tracked;
  for (int idx : unmatched_tracked_idx)
    remain_tracked.push_back(tracks_[idx]);

  std::vector<std::pair<int, int>> matches2;
  std::vector<int> still_unmatched_tracked_idx, unmatched_dlow_idx;
  associate(remain_tracked, d_low, match_dist_low_,
            matches2, still_unmatched_tracked_idx, unmatched_dlow_idx);

  for (const auto & m : matches2) {
    auto trk = remain_tracked[m.first];
    updateTrack(trk, *d_low[m.second]);
    output_tracked.push_back(trk);
  }

  // === Stage 3: Lost tracks vs remaining D_high ===
  std::vector<const lidar_interfaces::msg::Cone *> remain_dhigh;
  for (int idx : unmatched_dhigh_idx)
    remain_dhigh.push_back(d_high[idx]);

  std::vector<std::pair<int, int>> matches3;
  std::vector<int> unmatched_lost_idx, unmatched_dhigh2_idx;
  associate(lost_tracks_, remain_dhigh, match_dist_high_,
            matches3, unmatched_lost_idx, unmatched_dhigh2_idx);

  for (const auto & m : matches3) {
    auto trk = lost_tracks_[m.first];
    updateTrack(trk, *remain_dhigh[m.second]);
    output_tracked.push_back(trk);
  }

  // === Handle unmatched tracked → Lost ===
  std::vector<std::shared_ptr<Track>> new_lost;
  for (int idx : still_unmatched_tracked_idx) {
    auto trk = remain_tracked[idx];
    trk->state = Track::Lost;
    trk->time_since_update++;
    new_lost.push_back(trk);
  }

  // === Handle unmatched lost → keep or remove ===
  for (int idx : unmatched_lost_idx) {
    auto trk = lost_tracks_[idx];
    trk->time_since_update++;
    if (trk->time_since_update <= max_time_lost_)
      new_lost.push_back(trk);
  }

  // === New tracks from remaining unmatched D_high ===
  for (int idx : unmatched_dhigh2_idx) {
    const auto * det = remain_dhigh[idx];
    if (det->confidence < new_track_thresh_) continue;

    auto t = std::make_shared<Track>();
    t->id = next_id_++;
    t->kf = KalmanFilter(sigma_a_, sigma_r_);
    t->kf.init(static_cast<float>(det->position.x),
               static_cast<float>(det->position.y));
    t->z = static_cast<float>(det->position.z);
    t->dimensions = Eigen::Vector3f(
      static_cast<float>(det->dimensions.x),
      static_cast<float>(det->dimensions.y),
      static_cast<float>(det->dimensions.z));
    t->confidence = det->confidence;
    t->label = det->label;
    t->time_since_update = 0;
    t->hits = 1;
    t->state = Track::Tracked;
    output_tracked.push_back(t);
  }

  // === Rebuild track lists ===
  tracks_ = output_tracked;
  lost_tracks_ = new_lost;

  // === Generate output ===
  std::vector<lidar_interfaces::msg::TrackedCone> output;
  for (const auto & t : tracks_) {
    lidar_interfaces::msg::TrackedCone msg;
    msg.track_id = t->id;
    Eigen::Vector4f s = t->kf.getState();
    msg.position.x = s(0);
    msg.position.y = s(1);
    msg.position.z = t->z;
    msg.velocity.x = s(2);
    msg.velocity.y = s(3);
    msg.velocity.z = 0.0;
    msg.dimensions.x = t->dimensions.x();
    msg.dimensions.y = t->dimensions.y();
    msg.dimensions.z = t->dimensions.z();
    msg.confidence = t->confidence;
    msg.label = t->label;
    output.push_back(msg);
  }
  return output;
}

} // namespace lidar_tracking
