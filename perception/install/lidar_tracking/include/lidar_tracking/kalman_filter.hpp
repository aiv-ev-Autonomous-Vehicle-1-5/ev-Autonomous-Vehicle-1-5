#ifndef LIDAR_TRACKING_KALMAN_FILTER_HPP_
#define LIDAR_TRACKING_KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace lidar_tracking
{

class KalmanFilter
{
public:
  // sigma_a: acceleration noise (process), sigma_r: measurement noise
  explicit KalmanFilter(float sigma_a = 1.0f, float sigma_r = 0.1f);

  void init(float x, float y);
  void predict(float dt);
  void update(const Eigen::Vector2f & measurement);

  Eigen::Vector4f getState() const;   // [x, y, vx, vy]
  Eigen::Matrix4f getCovariance() const;

private:
  Eigen::Vector4f x_;          // state [x, y, vx, vy]
  Eigen::Matrix4f P_;          // covariance
  Eigen::Matrix<float, 2, 4> H_;  // measurement matrix
  Eigen::Matrix2f R_;          // measurement noise

  float sigma_a_;              // acceleration uncertainty (for Q computation)
};

} // namespace lidar_tracking

#endif // LIDAR_TRACKING_KALMAN_FILTER_HPP_
