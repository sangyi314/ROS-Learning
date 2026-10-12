#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/get_stats.hpp"

int main(int argc, char * argv[])
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stats_client");
  auto client = node->create_client<robot_interfaces::srv::GetStats>("sensor/get_stats");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service unavailable after 5 seconds");
    rclcpp::shutdown();
    return 1;
  }
  auto request = std::make_shared<robot_interfaces::srv::GetStats::Request>();
  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(node, future, 5s);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(future);
    RCLCPP_ERROR(node->get_logger(), "No response: timeout or interrupted");
    rclcpp::shutdown();
    return 2;
  }

  const auto response = future.get();
  RCLCPP_INFO(
    node->get_logger(), "accepted=%llu rejected=%llu has_data=%s latest=%.3f average=%.3f",
    static_cast<unsigned long long>(response->accepted_count),
    static_cast<unsigned long long>(response->rejected_count),
    response->has_data ? "true" : "false",
    response->latest_temperature_c, response->moving_average_c);
  rclcpp::shutdown();
  return 0;
}