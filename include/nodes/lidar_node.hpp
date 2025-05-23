//
// Created by student on 7.4.25.
//

#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP
#include <rclcpp/node.hpp>

#include "algorithms/lidar_filtr.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>

namespace nodes {
    class LidarNode  : public rclcpp::Node {
    public:
        LidarNode();

        ~LidarNode() override = default;

    private:

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_avg_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lidar_turn_publisher_;

        void on_lidar_msg(std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

        algorithms::LidarFiltrResults filtr_results_;
    };
}



#endif //LIDAR_NODE_HPP
