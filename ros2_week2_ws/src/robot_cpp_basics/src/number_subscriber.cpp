#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_cpp_basics/moving_average.hpp"
#include "std_msgs/msg/float64.hpp"

class NumberSubscriber : public rclcpp::Node
{
    public:
        NumberSubscriber() : Node("number_subscriber")
        {
            subscription_ = create_subscription<std_msgs::msg::Float64>("numbers" , 10 , [this](std_msgs::msg::Float64::ConstSharedPtr message){
                if(!std::isfinite(message -> data) || std::abs(message -> data) > 1000000.0)
                {
                    RCLCPP_WARN(get_logger() , "Rejected invalid or excessive value");
                    return;
                }
                const double average = filter_.push(message -> data);
                RCLCPP_INFO(get_logger() , "raw = %.1f , average = %.3f , window_size = %zu" , message -> data , average , filter_.size());
            });
        }

    private:
        MovingAverage filter_{3};
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc , char* argv[])
{
    rclcpp::init(argc , argv);
    rclcpp::spin(std::make_shared<NumberSubscriber>());
    rclcpp::shutdown();

    return 0;
}