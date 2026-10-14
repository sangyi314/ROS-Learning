#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace diff_drive_lab
{
struct BodyVelocity { double v{0.0}; double w{0.0}; };
struct WheelSpeeds { double left{0.0}; double right{0.0}; };
struct Pose2D { double x{0.0}; double y{0.0}; double yaw{0.0}; };

inline double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline WheelSpeeds limit_wheels(WheelSpeeds wheels, double limit)
{
  if (!std::isfinite(wheels.left) || !std::isfinite(wheels.right) ||
    !std::isfinite(limit) || limit <= 0.0)
  {
    throw std::invalid_argument("Finite wheel speeds and a positive limit are required");
  }
  const double largest = std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (largest > limit) {
    const double scale = limit / largest;
    wheels.left *= scale;
    wheels.right *= scale;
  }
  return wheels;
}

class DiffDrive
{
public:
  DiffDrive(double wheel_radius, double wheel_separation)
  : radius_(wheel_radius), separation_(wheel_separation)
  {
    if (!std::isfinite(radius_) || !std::isfinite(separation_) ||
      radius_ <= 0.0 || separation_ <= 0.0)
    {
      throw std::invalid_argument("Geometry must be finite and positive");
    }
  }

  WheelSpeeds inverse(BodyVelocity body) const
  {
    if (!std::isfinite(body.v) || !std::isfinite(body.w)) {
      throw std::invalid_argument("Body velocity must be finite");
    }
    return {(body.v - separation_ * body.w / 2.0) / radius_,
      (body.v + separation_ * body.w / 2.0) / radius_};
  }

  BodyVelocity forward(WheelSpeeds wheels) const
  {
    if (!std::isfinite(wheels.left) || !std::isfinite(wheels.right)) {
      throw std::invalid_argument("Wheel speeds must be finite");
    }
    return {radius_ * (wheels.left + wheels.right) / 2.0,
      radius_ * (wheels.right - wheels.left) / separation_};
  }

  void step(WheelSpeeds actual_wheels, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("dt must be finite and positive");
    }
    const auto body = forward(actual_wheels);
    const double distance = body.v * dt;
    const double turn = body.w * dt;
    const double half = turn / 2.0;
    const double half2 = half * half;
    const double sinc = std::abs(half) < 1e-6 ?
      1.0 - half2 / 6.0 + half2 * half2 / 120.0 : std::sin(half) / half;
    pose_.x += distance * sinc * std::cos(pose_.yaw + half);
    pose_.y += distance * sinc * std::sin(pose_.yaw + half);
    pose_.yaw = wrap_angle(pose_.yaw + turn);
    left_angle_ += actual_wheels.left * dt;
    right_angle_ += actual_wheels.right * dt;
  }

  void reset()
  {
    pose_ = {};
    left_angle_ = 0.0;
    right_angle_ = 0.0;
  }

  const Pose2D & pose() const { return pose_; }
  double left_angle() const { return left_angle_; }
  double right_angle() const { return right_angle_; }

private:
  double radius_;
  double separation_;
  Pose2D pose_{};
  double left_angle_{0.0};
  double right_angle_{0.0};
};
} 