#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class HelloNode : public rclcpp::Node
{
public:
  HelloNode() : Node("hello_node")
  {
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        ++count_;
        RCLCPP_INFO(
          get_logger(), "heartbeat=%llu",
          static_cast<unsigned long long>(count_));
      });
  }

private:
  std::uint64_t count_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloNode>());
  rclcpp::shutdown();
  return 0;
}