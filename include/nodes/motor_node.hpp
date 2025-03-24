//
// Created by student on 10.3.25.
//

#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        // Constructor
        MotorNode();
        // Destructor (default)
        ~MotorNode() override = default;

        double get_left_angular_velocity() const;
        double get_right_angular_velocity() const;

        void go_right();
        void go_left();
        void go_forward();
        void go(uint8_t l, uint8_t r);

        // TODO: private
        void motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish);
    private:

        // Variable to store the last received button press value
        uint32_t encoder_left_ = 0;
        uint32_t encoder_right_ = 0;

        uint32_t encoder_left_last_ = 0;
        uint32_t encoder_right_last_ = 0;

        // Angular velocity in rotations per second
        double wLeft = 0;
        double wRight = 0;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr motor_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;

        // Callback - preprocess received message
        void on_motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
        void timer_callback();

        double uptime_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Start time for uptime calculation
        rclcpp::Time last_time_;
        std_msgs::msg::UInt8MultiArray motor_message;
    };
}



#endif //MOTOR_NODE_HPP
