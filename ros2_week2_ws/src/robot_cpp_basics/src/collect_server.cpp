#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/collect_samples.hpp"
#include "robot_interfaces/msg/sensor_sample.hpp"

class CollectServer : public rclcpp::Node
{
public:
  using Action = robot_interfaces::action::CollectSamples;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;

  CollectServer() : Node("collect_server")
  {
    server_ = rclcpp_action::create_server<Action>(
      this, "sensor/collect_samples",
      [this](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Action::Goal> goal) {
        (void)uuid;
        if (goal->sample_count < 1 || goal->sample_count > 100 || goal_reserved_) {
          RCLCPP_WARN(get_logger(), "Goal rejected: invalid count or server busy");
          return rclcpp_action::GoalResponse::REJECT;
        }
        goal_reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        if (handle == active_goal_) {
          return rclcpp_action::CancelResponse::ACCEPT;
        }
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        active_goal_ = handle;
        collected_ = 0;
        sum_ = 0.0;
        last_valid_time_ = Clock::now();
        RCLCPP_INFO(
          get_logger(), "Goal accepted: collect %u valid samples",
          static_cast<unsigned>(handle->get_goal()->sample_count));
      });

    subscription_ = create_subscription<robot_interfaces::msg::SensorSample>(
      "sensor/raw", rclcpp::QoS(10).reliable(),
      [this](robot_interfaces::msg::SensorSample::ConstSharedPtr message) {
        if (!active_goal_ || active_goal_->is_canceling()) {
          return;
        }
        const double value = message->temperature_c;
        if (!message->valid || !std::isfinite(value) || value < -40.0 || value > 125.0) {
          return;
        }
        ++collected_;
        sum_ += value;
        last_valid_time_ = Clock::now();
        const auto target = active_goal_->get_goal()->sample_count;
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->collected_count = collected_;
        feedback->target_count = target;
        feedback->progress = static_cast<double>(collected_) / target;
        active_goal_->publish_feedback(feedback);
        if (collected_ >= target) {
          finish(Finish::Succeeded, "Requested samples collected");
        }
      });

    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!active_goal_) {
        return;
      }
      if (active_goal_->is_canceling()) {
        finish(Finish::Canceled, "Canceled by client; returning partial statistics");
        return;
      }
      if (Clock::now() - last_valid_time_ >= std::chrono::seconds(5)) {
        finish(Finish::Aborted, "No valid sample for 5 seconds");
      }
    });
  }

private:
  enum class Finish { Succeeded, Canceled, Aborted };

  void finish(Finish state, const char * message)
  {
    auto result = std::make_shared<Action::Result>();
    result->collected_count = collected_;
    result->mean_temperature_c = collected_ == 0 ? 0.0 : sum_ / collected_;
    result->message = message;
    if (state == Finish::Succeeded) {
      active_goal_->succeed(result);
    } else if (state == Finish::Canceled) {
      active_goal_->canceled(result);
    } else {
      active_goal_->abort(result);
    }
    RCLCPP_INFO(get_logger(), "%s; collected=%u", message, static_cast<unsigned>(collected_));
    active_goal_.reset();
    goal_reserved_ = false;
  }

  bool goal_reserved_{false};
  std::uint32_t collected_{0};
  double sum_{0.0};
  Clock::time_point last_valid_time_{};
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<robot_interfaces::msg::SensorSample>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectServer>());
  rclcpp::shutdown();
  return 0;
}