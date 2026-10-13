#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/collect_samples.hpp"

int run_client()
{
  using namespace std::chrono_literals;
  using Action = robot_interfaces::action::CollectSamples;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  auto node = std::make_shared<rclcpp::Node>("collect_client");
  const auto sample_count = node->declare_parameter<std::int64_t>("sample_count", 20);
  const auto cancel_after_ms = node->declare_parameter<std::int64_t>("cancel_after_ms", 0);
  if (sample_count < 1 || sample_count > 100) {
    throw std::invalid_argument("sample_count must be in [1, 100]");
  }
  if (cancel_after_ms < 0 || cancel_after_ms > 300000) {
    throw std::invalid_argument("cancel_after_ms must be in [0, 300000]; 0 disables it");
  }

  auto client = rclcpp_action::create_client<Action>(node, "sensor/collect_samples");
  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Action server unavailable after 5 seconds");
    return 1;
  }
  Action::Goal goal;
  goal.sample_count = static_cast<std::uint32_t>(sample_count);
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [node](
    GoalHandle::SharedPtr handle, const std::shared_ptr<const Action::Feedback> feedback) {
      (void)handle;
      RCLCPP_INFO(
        node->get_logger(), "feedback=%u/%u progress=%.1f%%",
        static_cast<unsigned>(feedback->collected_count),
        static_cast<unsigned>(feedback->target_count), feedback->progress * 100.0);
    };

  auto goal_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(), "No goal response; acceptance is unknown. Check the server.");
    return 2;
  }
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected by server");
    return 3;
  }
  auto result_future = client->async_get_result(goal_handle);
  rclcpp::TimerBase::SharedPtr cancel_timer;
  bool cancel_requested = false;
  if (cancel_after_ms > 0) {
    cancel_timer = node->create_wall_timer(
      std::chrono::milliseconds(cancel_after_ms), [&, client, goal_handle, node]() {
        cancel_timer->cancel();
        cancel_requested = true;
        (void)client->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node->get_logger(), "Cancel requested; waiting for final result");
      });
  }

  auto result_status = rclcpp::spin_until_future_complete(node, result_future, 300s);
  if (result_status == rclcpp::FutureReturnCode::TIMEOUT) {
    if (cancel_timer) {
      cancel_timer->cancel();
    }
    if (!cancel_requested) {
      (void)client->async_cancel_goal(goal_handle);
    }
    RCLCPP_WARN(node->get_logger(), "Client timeout; cancel requested, waiting 5 more seconds");
    result_status = rclcpp::spin_until_future_complete(node, result_future, 5s);
  }
  if (cancel_timer) {
    cancel_timer->cancel();
  }
  if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "No terminal result; remote task state is unknown");
    return 4;
  }

  const auto wrapped = result_future.get();
  if (!wrapped.result) {
    RCLCPP_ERROR(node->get_logger(), "Result payload unavailable");
    return 5;
  }
  const char * state = "UNKNOWN";
  int exit_code = 5;
  switch (wrapped.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      state = "SUCCEEDED";
      exit_code = 0;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      state = "CANCELED";
      exit_code = 0;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      state = "ABORTED";
      exit_code = 6;
      break;
    default:
      break;
  }
  RCLCPP_INFO(
    node->get_logger(), "%s collected=%u mean=%.3f message=%s",
    state, static_cast<unsigned>(wrapped.result->collected_count),
    wrapped.result->mean_temperature_c, wrapped.result->message.c_str());
  return exit_code;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    exit_code = run_client();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("collect_client"), "%s", error.what());
    exit_code = 7;
  }
  rclcpp::shutdown();
  return exit_code;
}