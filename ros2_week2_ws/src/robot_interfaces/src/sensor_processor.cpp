#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensor_sample.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_cpp_basics/moving_average.hpp"

class SensorProcessor : public rclcpp::Node
{
public:
  SensorProcessor() : Node("sensor_processor")
  {
    filtered_publisher_ = create_publisher<std_msgs::msg::Float64>("sensor/filtered", 10);
    subscription_ = create_subscription<robot_interfaces::msg::SensorSample>(
      "sensor/raw", rclcpp::QoS(10).reliable(),
      [this](robot_interfaces::msg::SensorSample::ConstSharedPtr sample) {
        if (!sample->valid || !std::isfinite(sample->temperature_c) ||
          sample->temperature_c < -40.0 || sample->temperature_c > 125.0)
        {
          RCLCPP_WARN(get_logger(), "Rejected sample seq=%llu",
            static_cast<unsigned long long>(sample->sequence));
          return;
        }
        std_msgs::msg::Float64 filtered;
        filtered.data = filter_.push(sample->temperature_c);
        filtered_publisher_->publish(filtered);
        RCLCPP_INFO(get_logger(), "raw=%.3f filtered=%.3f",
          sample->temperature_c, filtered.data);
      });
  }

private:
  MovingAverage filter_{3};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  rclcpp::Subscription<robot_interfaces::msg::SensorSample>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorProcessor>());
  rclcpp::shutdown();
  return 0;
}