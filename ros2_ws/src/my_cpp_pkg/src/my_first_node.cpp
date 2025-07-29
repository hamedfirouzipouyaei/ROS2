#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_cpp_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2! from a C++ class-based node");
        timer_ = this -> create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timer_callback, this)
        );
    }
private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hi there! This is a timer callback in C++ %d", count_);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0; // Example member variable
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create an instance of MyNode and spin it
    // This will keep the node alive and process callbacks
    // until rclcpp::shutdown() is called.
    rclcpp::spin(std::make_shared<MyNode>());

    // we can also do like following:
    // auto node = std::make_shared<MyNode>();
    // rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
