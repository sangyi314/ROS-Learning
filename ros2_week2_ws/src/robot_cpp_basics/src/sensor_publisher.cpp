#include <cmath>
#include <memory>
#include <chrono>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensor_sample.hpp"
#include "rcl_interfaces//msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" 

class SensorPublisher : public rclcpp::Node
{
    public:
        SensorPublisher() : Node("sensor_publisher")
        {
            rcl_interfaces::msg::ParameterDescriptor period_descriptor;
            period_descriptor.description = "Publish period in milliseconds: 20 ... 2000";
            period_descriptor.read_only = true ;
            const auto period_ms = declare_parameter<std::uint64_t>("period_ms" , 200 , period_descriptor);
            const auto base = declare_parameter<double>("base_temperature_c" , 25.0);
            const auto inject_invalid = declare_parameter<bool>("inject_invalid" , true);
            if (period_ms < 20 || period_ms > 2000) {
                throw std::invalid_argument("period_ms must be in [20, 2000]");
            }
            if (!std::isfinite(base) || base < 0.0 || base > 60.0) {
                throw std::invalid_argument("base_temperature_c must be in [0.0, 60.0]");
            }
            parameter_callback_ = add_on_set_parameters_callback(
            [](const std::vector<rclcpp::Parameter> & parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for(const auto& para: parameters)
                {
                    if(para.get_name() == "base_temperature_c")
                    {
                        if(para.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                        {
                            result.successful = false;
                            result.reason = "base_temperature_c must be a double";
                            return result;
                        }
                        const auto value = para.as_double();
                        if(!std::isfinite(value) || value < 0.0 || value > 60.0)
                        {
                            result.successful = false;
                            result.reason = "base_temperature_c must be in [0.0, 60.0]";
                            return result;
                        }
                    }
                    else if(para.get_name() == "period_ms")
                    {
                        if(para.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                        {
                            result.successful = false;
                            result.reason = "period_ms must be an integer";
                            return result;
                        }
                        const auto value = para.as_int();
                        if(value < 20 || value > 2000)
                        {
                            result.successful = false;
                            result.reason = "period_ms must be in [20, 2000]";
                            return result;
                        }
                    }
                }
            }
            );
            
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
            const double current_base = get_parameter("base_temperature_c").as_double();
            const bool inject_invalid = get_parameter("inject_invalid").as_bool();
            sample.temperature_c = current_base + 2.0 * std::sin(phase) + noise;
            sample.valid = (sequence_ % 10 != 0) || !inject_invalid;
            publisher_->publish(sample);
            RCLCPP_INFO(
            get_logger(), "seq=%llu temp=%.3f valid=%s",
            static_cast<unsigned long long>(sample.sequence), sample.temperature_c,
            sample.valid ? "true" : "false");
        }
        std::uint64_t sequence_{0};
        rclcpp::Publisher<robot_interfaces::msg::SensorSample>::SharedPtr publisher_; 
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

};

int main (int argc , char* argv[])
{
    rclcpp::init(argc , argv);
    try
    {
        rclcpp::spin(std::make_shared<SensorPublisher>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_publisher"), "Error occurred: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}