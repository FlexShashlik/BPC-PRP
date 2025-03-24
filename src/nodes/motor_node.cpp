//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.hpp"
#include "helper.hpp"
#include "../include/kinematics.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motorNode"), last_time_(this->now()) {
        this->get_logger().set_level(rclcpp::Logger::Level::Info);

        motor_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
             Topic::encoders, 1, std::bind(&MotorNode::on_motor_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);

        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(ENCODER_POLLING_RATE_MS)),
            std::bind(&MotorNode::timer_callback, this));

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

    void MotorNode::timer_callback() {
        RCLCPP_DEBUG(this->get_logger(), "Timer triggered. Publishing uptime...");

        auto dLeft = encoder_left_ - encoder_left_last_;
        auto dRight = encoder_right_last_ - encoder_right_; // because of encoder's overflow
        auto dTime = (this->now() - last_time_).seconds();

        RCLCPP_DEBUG(this->get_logger(), "dTime: %f", dTime);

        // Angular velocity in ticks/dT
        wLeft = (dLeft / dTime);
        wRight = (dRight / dTime);

        // Convert ang velocity to rotations per second
        wLeft *= 1./TICKS_PER_ROTATION;
        wRight *= 1./TICKS_PER_ROTATION;

        RCLCPP_DEBUG(this->get_logger(), "Left RPS: %f", wLeft);
        RCLCPP_DEBUG(this->get_logger(), "Right RPS: %f", wRight);

        double vL = kinematics::calc_wheel_speed(wLeft);
        double vR = kinematics::calc_wheel_speed(wRight);
        RCLCPP_DEBUG(this->get_logger(), "Left speed %f m/s", vL);
        RCLCPP_DEBUG(this->get_logger(), "Right speed %f m/s", vR);

        double dV = kinematics::calc_delta_speed(vL, vR);
        RCLCPP_DEBUG(this->get_logger(), "Delta speed %f m/s", dV);

        double dAngle = kinematics::calc_delta_angle(vL, vR);
        RCLCPP_DEBUG(this->get_logger(), "Delta angle: %f", dAngle);

        kinematics::calc_pos(dV, dAngle);
        RCLCPP_DEBUG(this->get_logger(), "X: %f", kinematics::xPos);
        RCLCPP_DEBUG(this->get_logger(), "Y: %f", kinematics::yPos);
        RCLCPP_DEBUG(this->get_logger(), "Angle: %f", kinematics::anglePos);

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

    void MotorNode::go_right() {
        motor_message.data = {127, MAX_MOTOR_SPEED};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go_left() {
        motor_message.data = {MAX_MOTOR_SPEED, 127};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go_forward() {
        motor_message.data = {MAX_MOTOR_SPEED, MAX_MOTOR_SPEED};
        this->motor_speeds_publish(motor_message);
    }

    void MotorNode::go(uint8_t l, uint8_t r) {
        motor_message.data = {l, r};
        this->motor_speeds_publish(motor_message);
    }
}
