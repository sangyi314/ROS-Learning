#include <cmath>
#include <memory>
#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensor_sample.hpp"

class SensorPublisher : public rclcpp::Node
{
    public:
        SensorPublisher() : Node("sensor_publisher")
        {
            publisher_ = create_publisher<robot_interfaces::msg::SensorSample>("sensor/raw" , rclcpp::QoS(10).reliable());
            timer_ = create_wall_timer(std::chrono::milliseconds(200) , [this](){
                publish_sample();
            });
        }
    private:
        void publish_sample()
        {
            robot_interfaces::msg::SensorSample sample;
            sample.header.stamp = now();
            sample.header.frame_id = "virtual_sensor";
            sample.sequence = ++sequence_;
            const double phase = static_cast<double>(sequence_ % 100) * 0.2;
            const double noise = (sequence_ % 2 == 0) ? -0.2 : 0.2;
            sample.temperature_c = 25.0 + 2.0 * std::sin(phase) + noise;
            sample.valid = (sequence_ % 10 != 0);
            publisher_->publish(sample);
            RCLCPP_INFO(
            get_logger(), "seq=%llu temp=%.3f valid=%s",
            static_cast<unsigned long long>(sample.sequence), sample.temperature_c,
            sample.valid ? "true" : "false");
        }
        std::uint64_t sequence_{0};
        rclcpp::Publisher<robot_interfaces::msg::SensorSample>::SharedPtr publisher_; 
        rclcpp::TimerBase::SharedPtr timer_;

};

int main (int argc , char* argv[])
{
    rclcpp::init(argc , argv);
    rclcpp::spin(std::make_shared<SensorPublisher>());
    rclcpp::shutdown();
    return 0;
}