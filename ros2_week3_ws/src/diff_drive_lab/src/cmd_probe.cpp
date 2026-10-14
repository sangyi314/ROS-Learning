#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "diff_drive_lab/command_buffer.hpp"

class CmdProbe : public rclcpp::Node
{
public:
  CmdProbe() : Node("cmd_probe")
  {
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        const std::array<double, 4> unused{
          message->linear.y, message->linear.z, message->angular.x, message->angular.y};
        const bool planar = std::all_of(unused.begin(), unused.end(), [](double value) {
          return std::isfinite(value) && std::abs(value) <= 1e-9;
        });
        if (!planar || !commands_.accept(
            message->linear.x, message->angular.z, diff_drive_lab::steady_seconds()))
        {
          commands_.clear();
          RCLCPP_WARN(get_logger(), "Invalid planar command: stop");
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      const auto command = commands_.sample(diff_drive_lab::steady_seconds());
      RCLCPP_INFO(get_logger(), "applied v=%.3f m/s w=%.3f rad/s", command.v, command.w);
    });
  }

private:
  diff_drive_lab::CommandBuffer commands_{0.5, 2.0, 0.5};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdProbe>());
  rclcpp::shutdown();
  return 0;
}