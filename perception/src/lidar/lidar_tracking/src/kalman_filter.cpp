#include "lidar_tracking/kalman_filter.hpp"

namespace lidar_tracking
{

KalmanFilter::KalmanFilter(float sigma_a, float sigma_r)
  : sigma_a_(sigma_a)
{
  x_.setZero();
  P_.setIdentity();

  // H: observe [x, y] from [x, y, vx, vy]
  H_.setZero();
  H_(0, 0) = 1.0f;
  H_(1, 1) = 1.0f;

  // Measurement noise
  R_ = Eigen::Matrix2f::Identity() * (sigma_r * sigma_r);
}

void KalmanFilter::init(float x, float y)
{
  x_ << x, y, 0.0f, 0.0f;
  P_.setIdentity();
  P_(2, 2) = 10.0f;  // high uncertainty on initial velocity
  P_(3, 3) = 10.0f;
}

void KalmanFilter::predict(float dt)
{
  // F: constant velocity model
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  // Q: process noise from piecewise constant acceleration model
  //   G = [dt^2/2, dt^2/2, dt, dt]^T
  //   Q = G * G^T * sigma_a^2
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  float sa2 = sigma_a_ * sigma_a_;

  Eigen::Matrix4f Q;
  Q << dt4/4, 0,     dt3/2, 0,
       0,     dt4/4, 0,     dt3/2,
       dt3/2, 0,     dt2,   0,
       0,     dt3/2, 0,     dt2;
  Q *= sa2;

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Vector2f & z)
{
  Eigen::Vector2f y = z - H_ * x_;
  Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<float, 4, 2> K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (Eigen::Matrix4f::Identity() - K * H_) * P_;
}

Eigen::Vector4f KalmanFilter::getState() const { return x_; }
Eigen::Matrix4f KalmanFilter::getCovariance() const { return P_; }

} // namespace lidar_tracking
