#include <cmath>
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensor_sample.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_cpp_basics/moving_average.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "robot_interfaces/srv/get_stats.hpp"
#include "std_srvs/srv/trigger.hpp"

class SensorProcessor : public rclcpp::Node
{
public:
  SensorProcessor() : Node("sensor_processor")
  {
    rcl_interfaces::msg::ParameterDescriptor description;
    description.description = "Moving average window: 1 ... 50";
    description.read_only = true;
    const auto window = declare_parameter<std::int64_t>("window_size" , 3 , description);
    if (window < 1 || window > 50)
    {
      throw std::invalid_argument("window size must be in [1, 50]");
    }

    filter_ = std::make_unique<MovingAverage>(static_cast<std::size_t>(window));

    filtered_publisher_ = create_publisher<std_msgs::msg::Float64>("sensor/filtered", 10);
    subscription_ = create_subscription<robot_interfaces::msg::SensorSample>(
      "sensor/raw", rclcpp::QoS(10).reliable(),
      [this](robot_interfaces::msg::SensorSample::ConstSharedPtr sample) {
        process(*sample);
      });
      stats_service_ = create_service<robot_interfaces::srv::GetStats>(
        "sensor/get_stats",
        [this](robot_interfaces::srv::GetStats::Request::SharedPtr request,
               robot_interfaces::srv::GetStats::Response::SharedPtr response) {
          (void)request;
          response->has_data = has_data_;
          response->latest_temperature_c = latest_temperature_c_;
          response->moving_average_c = moving_average_c_;
          response->accepted_count = accepted_count_;
          response->rejected_count = rejected_count_;
        });
      reset_service_ = create_service<std_srvs::srv::Trigger>(
        "sensor/reset_stats",
        [this](std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          (void)request;
          has_data_ = false;
          latest_temperature_c_ = 0.0;
          moving_average_c_ = 0.0;
          rejected_count_ = 0;
          accepted_count_ = 0;
          response->success = true;
          response->message = "Statistics reset successfully.";
        });
        
  }

private:
  void process(const robot_interfaces::msg::SensorSample & message)
  {
    const double value = message.temperature_c;
    if (!message.valid || !std::isfinite(value) || value < -40.0 || value > 125.0) {
      ++rejected_count_;
      RCLCPP_WARN(
        get_logger(), "rejected seq=%llu rejected_count=%llu",
        static_cast<unsigned long long>(message.sequence),
        static_cast<unsigned long long>(rejected_count_));
      return;
    }
    ++accepted_count_;
    has_data_ = true;
    latest_temperature_c_ = value;
    moving_average_c_ = filter_->push(value);
    std_msgs::msg::Float64 filtered;
    filtered.data = moving_average_c_;
    filtered_publisher_->publish(filtered);
    RCLCPP_INFO(
       get_logger(), "seq=%llu raw=%.3f average=%.3f accepted=%llu",
      static_cast<unsigned long long>(message.sequence), value, moving_average_c_,
      static_cast<unsigned long long>(accepted_count_));
  }
  bool has_data_{false};
  double latest_temperature_c_{0.0};
  double moving_average_c_{0.0};
  std::uint64_t rejected_count_{0};
  std::uint64_t accepted_count_{0};
  std::unique_ptr<MovingAverage> filter_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  rclcpp::Subscription<robot_interfaces::msg::SensorSample>::SharedPtr subscription_;
  rclcpp::Service<robot_interfaces::srv::GetStats>::SharedPtr stats_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SensorProcessor>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_processor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
