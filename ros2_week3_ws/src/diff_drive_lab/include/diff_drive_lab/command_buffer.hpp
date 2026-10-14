#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "diff_drive_lab/diff_drive.hpp"

namespace diff_drive_lab
{
inline double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class CommandBuffer
{
public:
  CommandBuffer(double max_v, double max_w, double timeout)
  : max_v_(max_v), max_w_(max_w), timeout_(timeout)
  {
    if (!std::isfinite(max_v_) || !std::isfinite(max_w_) ||
      !std::isfinite(timeout_) || max_v_ <= 0.0 || max_w_ <= 0.0 || timeout_ <= 0.0)
    {
      throw std::invalid_argument("Limits and timeout must be finite and positive");
    }
  }

  bool accept(double v, double w, double received_at)
  {
    if (!std::isfinite(v) || !std::isfinite(w) || !std::isfinite(received_at)) {
      clear();
      return false;
    }
    command_ = {std::clamp(v, -max_v_, max_v_), std::clamp(w, -max_w_, max_w_)};
    received_at_ = received_at;
    valid_ = true;
    return true;
  }

  BodyVelocity sample(double now) const
  {
    if (!valid_ || !std::isfinite(now) || now < received_at_ ||
      now - received_at_ > timeout_)
    {
      return {};
    }
    return command_;
  }

  void clear() { valid_ = false; command_ = {}; }

private:
  double max_v_;
  double max_w_;
  double timeout_;
  bool valid_{false};
  double received_at_{0.0};
  BodyVelocity command_{};
};
}  