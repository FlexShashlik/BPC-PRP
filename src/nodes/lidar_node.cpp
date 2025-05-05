//
// Created by student on 7.4.25.
//

#include "../../include/nodes/lidar_node.hpp"

#include "helper.hpp"
#include "algorithms/lidar_filtr.hpp"

nodes::LidarNode::LidarNode() : Node("lidarNode") {
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
         Topic::lidar, 1, std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1));
}

algorithms::LidarFiltrResults nodes::LidarNode::GetLidarFiltrResults() {
    return filtr_results_;
}

void nodes::LidarNode::on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg) {
        filtr_results_ = algorithms::LidarFiltr::apply_filter(msg->ranges, msg->angle_min, msg->angle_max);

        RCLCPP_WARN(this->get_logger(), "LIDAR :: F:%f B:%f L:%f R:%f FL:%f FR: %f", filtr_results_.front, filtr_results_.back, filtr_results_.left, filtr_results_.right, filtr_results_.front_left, filtr_results_.front_right);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received empty LaserScan message.");
    }
}

