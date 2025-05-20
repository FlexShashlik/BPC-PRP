#ifndef IO_NODE_HPP
#define IO_NODE_HPP
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace nodes {
    enum LedType
    {
        Front,
        Left,
        Right,
        Back,
        FrontLeft,
        FrontRight,
    };
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retireve the last pressed button value
        int get_button_pressed() const;
        void rgb_led_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish) const;
    private:

        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Subscription<LedType>::SharedPtr turn_event_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8::SharedPtr msg);
        void on_turn_callback(LedType type);
    };
}

#endif //IO_NODE_HPP
