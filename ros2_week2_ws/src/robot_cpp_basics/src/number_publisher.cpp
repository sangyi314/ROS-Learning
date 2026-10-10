#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class NumberPublisher : public rclcpp::Node
{
    public:
        NumberPublisher() : Node("number_publisher")
        {
            publisher_ = create_publisher<std_msgs::msg::Float64>("numbers", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(500) , [this]()
                {
                    publish_number();
                });
        }
    private:
        void publish_number()
        {
            std_msgs::msg::Float64 message;
            message.data = static_cast<double>((sequence % 5)+1);
            sequence++;
            publisher_ -> publish(message);
            RCLCPP_INFO(get_logger() , "published = %.lf" , message.data);
        }
        std::uint64_t sequence{0};
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc , char* argv [])
{
    rclcpp::init(argc , argv);
    rclcpp::spin(std::make_shared<NumberPublisher>());
    rclcpp::shutdown();
    return 0;
}