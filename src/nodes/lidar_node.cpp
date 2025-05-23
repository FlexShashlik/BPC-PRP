//
// Created by student on 7.4.25.
//

#include "../../include/nodes/lidar_node.hpp"

#include "helper.hpp"
#include "algorithms/lidar_filtr.hpp"

nodes::LidarNode::LidarNode() : Node("lidarNode") {
    this->get_logger().set_level(rclcpp::Logger::Level::Warn);
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>
    (
        Topic::lidar,
        1,
        std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1)
    );

    // Create a publisher for the averaged measurements (Order: front, right, left, wide right, wide left, is right open, is left open, is right closed, is left closed, is front far enough for pid).
    lidar_avg_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>
    (
        Topic::lidar_avg,
        10
    );

    // Create a publisher for the detected turn type
    lidar_turn_publisher_ = this->create_publisher<std_msgs::msg::Int8>
    (
        Topic::detected_turn_type,
        10
    );
}

void nodes::LidarNode::on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg) {
        filtr_results_ = algorithms::LidarFiltr::apply_filter(msg->ranges, msg->angle_min, msg->angle_max);

        std_msgs::msg::Float32MultiArray avg_msg;
        avg_msg.data.resize(10);
        avg_msg.data[0] = filtr_results_.front;
        avg_msg.data[1] = filtr_results_.right;
        avg_msg.data[2] = filtr_results_.left;
        avg_msg.data[3] = filtr_results_.wide_right;
        avg_msg.data[4] = filtr_results_.wide_left;
        avg_msg.data[5] = static_cast<float>(filtr_results_.isRightOpen);
        avg_msg.data[6] = static_cast<float>(filtr_results_.isLeftOpen);
        avg_msg.data[7] = static_cast<float>(filtr_results_.isRightClosed);
        avg_msg.data[8] = static_cast<float>(filtr_results_.isLeftClosed);
        avg_msg.data[9] = static_cast<float>(filtr_results_.isFrontWallFarEnoughToUsePIDForCorridorFollowing);

        lidar_avg_publisher_->publish(avg_msg);

        RCLCPP_WARN
        (
            this->get_logger(),
            "LIDAR :: F:%.4f B:%.4f L:%.4f R:%.4f FL:%.4f FR:%.4f WL:%.4f WR:%.4f",
            filtr_results_.front,
            filtr_results_.back,
            filtr_results_.left,
            filtr_results_.right,
            filtr_results_.front_left,
            filtr_results_.front_right,
            filtr_results_.wide_left,
            filtr_results_.wide_right
        );
    } else {
        RCLCPP_WARN(this->get_logger(), "Received empty LaserScan message.");
    }
}
