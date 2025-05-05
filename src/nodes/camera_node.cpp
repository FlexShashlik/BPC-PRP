//
// Created by student on 5.5.25.
//

#include "../../include/nodes/camera_node.hpp"
#include <opencv2/opencv.hpp>
#include "helper.hpp"

nodes::CameraNode::CameraNode() : Node("camera") {
    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
         Topic::camera, 1, std::bind(&CameraNode::on_camera_msg, this, std::placeholders::_1));

    aruco_detector_ = algorithms::ArucoDetector();
}

void nodes::CameraNode::on_camera_msg(std::shared_ptr<sensor_msgs::msg::CompressedImage> msg) {
    // Convert the vector<uint8_t> into a cv::Mat
    const cv::Mat data(msg->data);  // This doesn't copy the data, just wraps it

    // Decode the image data into a color image
    const cv::Mat image = cv::imdecode(data, cv::IMREAD_COLOR);

    if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to decode image");
        return;
    }

    auto arucos = aruco_detector_.detect(image);

    for (const auto& aruco : arucos) {
        bool exists = std::any_of(Arucos.begin(), Arucos.end(),
                              [&aruco](const ArucoType& existingType) {
                                  return existingType == aruco.id;
                              });
        if (!exists) {
            Arucos.push_back(static_cast<ArucoType>(aruco.id));
        }
    }
}
