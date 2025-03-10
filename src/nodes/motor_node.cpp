//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.hpp"
#include "helper.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motorNode") {
        // button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        //     Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);
    }

    void MotorNode::motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) {
        motor_publisher_->publish(value_to_publish);
        RCLCPP_INFO(this->get_logger(), "Published leds");
    }
}
