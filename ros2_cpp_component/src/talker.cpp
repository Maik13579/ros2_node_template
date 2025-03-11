
#include "talker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <cinttypes>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace demo
{
    Talker::Talker(const rclcpp::NodeOptions & options)
    : Node("talker", options)
    {
    // Declare and get the publish rate parameter (default: 1.0 Hz).
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("data", "Hello World");
    this->get_parameter("publish_rate", publish_rate_);

    // Create a publisher on the topic "~/custom_topic" (remapped in launch).
    publisher_ = this->create_publisher<ros2_cpp_component_interfaces::msg::CustomMsg>("~/chatter", 10);

    // Create a wall timer that calls the member function timer_callback.
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&Talker::timer_callback, this)
    );
    }

    void Talker::timer_callback()
    {
    // Allocate a new message using a unique pointer.
    auto msg = std::make_unique<ros2_cpp_component_interfaces::msg::CustomMsg>();
    msg->data = this->get_parameter("data").as_string();

    // Print the message data and memory address.
    printf("Publishing message with data: %s, at address: 0x%" PRIxPTR "\n",
            msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));

    // Publish the message (ownership transferred via std::move).
    publisher_->publish(std::move(msg));
    }

} // namespace demo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Talker)
