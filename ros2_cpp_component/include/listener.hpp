#ifndef LISTENER_HPP
#define LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_component_interfaces/msg/custom_msg.hpp"

namespace demo
{

    class Listener : public rclcpp::Node
    {
        public:
        explicit Listener(const rclcpp::NodeOptions & options);
        private:
        void topic_callback(const ros2_cpp_component_interfaces::msg::CustomMsg::SharedPtr msg);
        rclcpp::Subscription<ros2_cpp_component_interfaces::msg::CustomMsg>::SharedPtr subscription_;
    };

} // namespace demo

#endif // LISTENER_HPP
