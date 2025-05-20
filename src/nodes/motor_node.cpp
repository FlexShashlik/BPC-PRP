//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.hpp"
#include "helper.hpp"
#include "../include/kinematics.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motorNode"), last_time_(this->now()) {
        this->get_logger().set_level(rclcpp::Logger::Level::Warn);

        motor_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
             Topic::encoders, 1, std::bind(&MotorNode::on_motor_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);

        motor_message = std_msgs::msg::UInt8MultiArray();
        motor_message.layout.dim.resize(1);
        motor_message.layout.dim[0].label = "SPEEDs";
        motor_message.layout.dim[0].size = 2;
        motor_message.layout.dim[0].stride = 2;
        motor_message.layout.data_offset = 0;
    }

    // [1] - left motor
    // [2] - right motor
    void MotorNode::motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) {
        motor_publisher_->publish(value_to_publish);
    }

    void MotorNode::on_motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            encoder_left_ = msg->data[0];
            encoder_right_ = msg->data[1];
            RCLCPP_DEBUG(this->get_logger(), "Enocder_left: %u", this->encoder_left_);
            RCLCPP_DEBUG(this->get_logger(), "Enocder_right: %u", this->encoder_right_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty UInt32MultiArray message.");
        }
    }

    void MotorNode::start() {
        isStopped_ = false;
    }

    void MotorNode::stop() {
        isStopped_ = true;
    }

    double MotorNode::get_left_angular_velocity() const {
        return wLeft;
    }
    double MotorNode::get_right_angular_velocity() const {
        return wRight;
    }

    void MotorNode::go_left() {
        if (isStopped_)
            return;
        motor_message.data = {127, MAX_MOTOR_SPEED};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go_right() {
        if (isStopped_)
            return;
        motor_message.data = {MAX_MOTOR_SPEED, 127};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go_forward() {
        if (isStopped_)
            return;
        motor_message.data = {MAX_MOTOR_SPEED, MAX_MOTOR_SPEED};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go(uint8_t l, uint8_t r) {
        if (isStopped_)
            return;
        motor_message.data = {l, r};
        this->motor_speeds_publish(motor_message);
    }
}
