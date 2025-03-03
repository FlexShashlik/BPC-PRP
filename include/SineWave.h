//
// Created by student on 3.3.25.
//

#ifndef SINEWAVE_H
#define SINEWAVE_H
#include <string>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>

class SineWave {
    public:
        // Constructor takes a shared_ptr to an existing node instead of creating one.
        SineWave(const rclcpp::Node::SharedPtr &node, const std::string &topic, const std::string &inputDataTopic)
            : node_(node) {

            // Initialize the publisher
            publisher_ = node_->create_publisher<std_msgs::msg::Float32>(topic, 1);

            // Initialize the subscriber
            subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
                inputDataTopic, 1, std::bind(&SineWave::subscriber_callback, this, std::placeholders::_1));

            RCLCPP_INFO(node_->get_logger(), "Node setup complete for topic: %s", topic.c_str());
        }

    private:
        void subscriber_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "Received: %f", msg->data);
            publish_message(std::sin(2*M_PI*msg->data));
        }

        void publish_message(float value_to_publish) {
            auto msg = std_msgs::msg::Float32();
            msg.data = value_to_publish;
            publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "Published: %f", msg.data);
        }

        // Shared pointer to the main ROS node
        rclcpp::Node::SharedPtr node_;

        // Publisher, subscriber, and timer
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
};
#endif //SINEWAVE_H
