#ifndef TALKER_HPP
#define TALKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_component_interfaces/msg/custom_msg.hpp"

namespace demo
{
    class Talker : public rclcpp::Node
    {
    public:
    explicit Talker(const rclcpp::NodeOptions & options);
    private:
    void timer_callback();
    rclcpp::Publisher<ros2_cpp_component_interfaces::msg::CustomMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double publish_rate_;
    std::string data_;
    int count_;
    };

} // namespace demo
#endif // TALKER_HPP
