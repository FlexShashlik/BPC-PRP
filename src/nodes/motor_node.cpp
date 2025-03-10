//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.hpp"
#include "helper.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motorNode"), last_time_(this->now()) {
         motor_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
             Topic::encoders, 1, std::bind(&MotorNode::on_motor_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);

        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(ENCODER_POLLING_RATE_MS)),
            std::bind(&MotorNode::timer_callback, this));
    }

    // [1] - left motor
    // [2] - right motor
    void MotorNode::motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) {
        motor_publisher_->publish(value_to_publish);
    }

    void MotorNode::on_motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        encoder_left_ = msg->data[0];
        encoder_right_ = msg->data[1];
        RCLCPP_INFO(this->get_logger(), "Enocder_left: %i", this->encoder_left_);
        RCLCPP_INFO(this->get_logger(), "Enocder_right: %i", this->encoder_right_);
    }

    void MotorNode::timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer triggered. Publishing uptime...");

        auto dLeft = encoder_left_ - encoder_left_last_;
        auto dRight = encoder_right_ - encoder_right_last_;
        auto dTime = (this->now() - last_time_).seconds();

        // Angular velocity in ticks/dT
        wLeft = (dLeft / dTime);
        wRight = (dRight / dTime);

        // Convert ang velocity to rotations per second
        wLeft *= 1./TICKS_PER_ROTATION;
        wRight *= 1./TICKS_PER_ROTATION;

        RCLCPP_INFO(this->get_logger(), "Left RPS: %f", wLeft);
        RCLCPP_INFO(this->get_logger(), "Right RPS: %f", wRight);

        encoder_left_last_ = encoder_left_;
        encoder_right_last_ = encoder_right_;
        last_time_ = this->now();
    }


    double MotorNode::get_left_angular_velocity() const {
        return wLeft;
    }
    double MotorNode::get_right_angular_velocity() const {
        return wRight;
    }
}
