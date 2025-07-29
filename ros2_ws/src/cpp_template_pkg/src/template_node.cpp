#include "rclcpp/rclcpp.hpp"

class TemplateNode : public rclcpp::Node
{
public:
    TemplateNode() : Node("my_cpp_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello, This is C++ Template Node");
        // You cann add other initializations here
        // For example, creating publishers, subscribers, services, timers, etc.
        // Here we create a timer that calls the timer_callback every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TemplateNode::timer_callback, this)
        );
    }
private:
    int count_ = 0; // Example member variable
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Counter %d", count_);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateNode>());
    rclcpp::shutdown();
    return 0;
}