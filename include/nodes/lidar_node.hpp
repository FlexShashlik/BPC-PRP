//
// Created by student on 7.4.25.
//

#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP
#include <rclcpp/node.hpp>

#include "algorithms/lidar_filtr.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nodes {
    class LidarNode  : public rclcpp::Node {
    public:
        LidarNode();

        ~LidarNode() override = default;

        algorithms::LidarFiltrResults GetLidarFiltrResults();

    private:

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

        void on_lidar_msg(std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

        algorithms::LidarFiltrResults filtr_results_;
    };
}



#endif //LIDAR_NODE_HPP
