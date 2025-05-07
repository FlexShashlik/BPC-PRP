//
// Created by student on 5.5.25.
//

#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "helper.hpp"
#include "algorithms/aruco_detector.hpp"

namespace nodes {
    class CameraNode : public rclcpp::Node{
    public:
        CameraNode();

        ~CameraNode() override = default;

        ArucoType GetNextMove() const;

    private:

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;

        void on_camera_msg(std::shared_ptr<sensor_msgs::msg::CompressedImage> msg);

        algorithms::ArucoDetector aruco_detector_;
        std::vector<ArucoType> arucos_;

        bool isTreasureDetected_;
    };
}


#endif //CAMERA_NODE_HPP
