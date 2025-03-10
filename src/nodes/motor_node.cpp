//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.hpp"
#include "helper.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motorNode") {
         motor_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
             Topic::encoders, 1, std::bind(&MotorNode::on_motor_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);
    }

    // [1] - left motor
    // [2] - right motor
    void MotorNode::motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) {
        motor_publisher_->publish(value_to_publish);
        RCLCPP_INFO(this->get_logger(), "Published leds");
    }

    void MotorNode::on_motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoder_left_ = msg->data[0];
        encoder_right_ = msg->data[1];
        RCLCPP_INFO(this->get_logger(), "Enocder_left: %i", this->encoder_left_);
        RCLCPP_INFO(this->get_logger(), "Enocder_right: %i", this->encoder_right_);
    }
}
