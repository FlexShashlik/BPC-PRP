#include "nodes/io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode() : Node("ioNode") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
        /*turn_event_subscriber_ = this->create_subscription<LedType>(
            Topic::turn_event, 1, std::bind(&IoNode::on_turn_callback, this, std::placeholders::_1));*/
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 1);
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Button pressed: %i", this->get_button_pressed());
    }

    void IoNode::on_turn_callback(const LedType type)
    {
        return;
    }

    void IoNode::rgb_led_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) const
    {
        led_publisher_->publish(value_to_publish);
    }
}