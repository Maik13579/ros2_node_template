#include "listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cinttypes>

namespace demo
{

    Listener::Listener(const rclcpp::NodeOptions & options)
    : Node("listener", options)
    {
    // Create a subscription on the topic "~custom_topic" (remapped in launch).
    subscription_ = this->create_subscription<ros2_cpp_component_interfaces::msg::CustomMsg>(
        "~/chatter", 10,
        std::bind(&Listener::topic_callback, this, std::placeholders::_1)
    );
    }

    void Listener::topic_callback(const ros2_cpp_component_interfaces::msg::CustomMsg::SharedPtr msg)
    {
    // Log the received message data and its memory address.
    printf("Received message with data: %s, at address: 0x%" PRIxPTR "\n\n",
            msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));
    }

} // namespace demo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Listener)
