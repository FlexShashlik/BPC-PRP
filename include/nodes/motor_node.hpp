//
// Created by student on 10.3.25.
//

#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        // Constructor
        MotorNode();
        // Destructor (default)
        ~MotorNode() override = default;

        // Function to retireve the last pressed button value
        //int get_button_pressed() const;
        void motor_speeds_publish(const std_msgs::msg::UInt8MultiArray &value_to_publish);
    private:

        // Variable to store the last received button press value
        //int button_pressed_ = -1;

        // Subscriber for button press messages
        //rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;

        // Callback - preprocess received message
        //void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    };
}



#endif //MOTOR_NODE_HPP
