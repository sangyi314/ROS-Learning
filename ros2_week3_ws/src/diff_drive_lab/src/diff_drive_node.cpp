#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "diff_drive_lab/command_buffer.hpp"
#include "diff_drive_lab/diff_drive.hpp"

class DiffDriveNode : public rclcpp::Node
{
public:
  DiffDriveNode() : Node("diff_drive_node")
  {
    const double radius = setting("wheel_radius_m", 0.05, 0.01, 0.5);
    const double separation = setting("wheel_separation_m", 0.30, 0.05, 2.0);
    const double rate = setting("update_rate_hz", 50.0, 20.0, 200.0);
    const double max_v = setting("max_linear_mps", 0.5, 0.01, 2.0);
    const double max_w = setting("max_angular_rps", 2.0, 0.01, 6.0);
    max_wheel_ = setting("max_wheel_rps", 12.0, 0.1, 50.0);
    const double timeout = setting("command_timeout_s", 0.5, 0.1, 2.0);
    max_step_ = setting("max_step_s", 0.1, 2.0 / rate, 0.5);
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom", fixed);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", fixed);
    publish_tf_ = declare_parameter<bool>("publish_tf", true, fixed);
    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_ ||
      odom_frame_.front() == '/' || base_frame_.front() == '/')
    {
      throw std::invalid_argument("Frame IDs must be nonempty, distinct and have no leading slash");
    }
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("This wall-time simulator requires use_sim_time=false");
    }
    clock_guard_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "use_sim_time" &&
            (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL ||
            parameter.as_bool()))
          {
            result.successful = false;
            result.reason = "Live simulator uses wall time; keep use_sim_time=false";
          }
        }
        return result;
      });
    model_ = std::make_unique<diff_drive_lab::DiffDrive>(radius, separation);
    commands_ = std::make_unique<diff_drive_lab::CommandBuffer>(max_v, max_w, timeout);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        const std::array<double, 4> unused{
          message->linear.y, message->linear.z, message->angular.x, message->angular.y};
        const bool planar = std::all_of(unused.begin(), unused.end(), [](double value) {
          return std::isfinite(value) && std::abs(value) <= 1e-9;
        });
        if (!planar || !commands_->accept(
            message->linear.x, message->angular.z, diff_drive_lab::steady_seconds()))
        {
          commands_->clear();
          RCLCPP_WARN(get_logger(), "Rejected invalid planar command; stopping");
        }
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset_odometry",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        model_->reset();
        commands_->clear();
        last_tick_ = diff_drive_lab::steady_seconds();
        response->success = true;
        response->message = "Pose and wheel angles reset; command buffer cleared";
      });
    last_tick_ = diff_drive_lab::steady_seconds();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, [this]() { tick(); });
  }

private:
  double setting(const std::string & name, double default_value, double low, double high)
  {
    rcl_interfaces::msg::ParameterDescriptor description;
    description.read_only = true;
    const double value = declare_parameter<double>(name, default_value, description);
    if (!std::isfinite(value) || value < low || value > high) {
      throw std::invalid_argument(name + " is outside its allowed range");
    }
    return value;
  }

  void tick()
  {
    const double current = diff_drive_lab::steady_seconds();
    const double dt = current - last_tick_;
    last_tick_ = current;
    diff_drive_lab::WheelSpeeds actual_wheels{};
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_step_) {
      commands_->clear();
      RCLCPP_WARN(get_logger(), "Bad update gap %.4f s: skip integration and stop", dt);
    } else {
      const auto requested = commands_->sample(current);
      actual_wheels = diff_drive_lab::limit_wheels(model_->inverse(requested), max_wheel_);
      model_->step(actual_wheels, dt);
    }
    publish_state(actual_wheels);
  }

  void publish_state(diff_drive_lab::WheelSpeeds actual_wheels)
  {
    const auto stamp = now();
    const auto pose = model_->pose();
    const auto body = model_->forward(actual_wheels);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, pose.yaw);
    orientation.normalize();

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = pose.x;
    odometry.pose.pose.position.y = pose.y;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = body.v;
    odometry.twist.twist.angular.z = body.w;
    const std::array<double, 6> pose_variances{0.01, 0.01, 1e6, 1e6, 1e6, 0.04};
    const std::array<double, 6> twist_variances{0.01, 1e6, 1e6, 1e6, 1e6, 0.04};
    for (std::size_t i = 0; i < 6; ++i) {
      odometry.pose.covariance[i * 6 + i] = pose_variances[i];
      odometry.twist.covariance[i * 6 + i] = twist_variances[i];
    }
    odom_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = odometry.pose.pose.orientation;
      broadcaster_->sendTransform(transform);
    }

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"left_wheel_joint", "right_wheel_joint"};
    joints.position = {model_->left_angle(), model_->right_angle()};
    joints.velocity = {actual_wheels.left, actual_wheels.right};
    joint_publisher_->publish(joints);
  }

  std::unique_ptr<diff_drive_lab::DiffDrive> model_;
  std::unique_ptr<diff_drive_lab::CommandBuffer> commands_;
  double max_wheel_{12.0};
  double max_step_{0.1};
  double last_tick_{0.0};
  bool publish_tf_{true};
  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr clock_guard_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DiffDriveNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("diff_drive_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}