//
// Created by student on 24.3.25.
//

#ifndef LINE_LOOP_HPP
#define LINE_LOOP_HPP
#include <memory>

#include "algorithms/pid.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/lidar_node.hpp"

namespace nodes {
    class ImuNode;
}

enum class LineLoopState {
    CALIBRATION,
    CORRIDOR_FOLLOWING,
    TURNING
};

class LineLoop : public rclcpp::Node {
public:
    // Constructor
    LineLoop(std::shared_ptr<nodes::CameraNode> camera, std::shared_ptr<nodes::ImuNode> imu, std::shared_ptr<nodes::LidarNode> lidar, std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor);
    // Destructor (default)
    ~LineLoop() override = default;
    void Restart();

    LineLoopState getState() const;

private:
    algorithms::Pid pid_;
    void line_loop_timer_callback();

    std::shared_ptr<nodes::LineNode> line_sensors_;
    std::shared_ptr<nodes::LidarNode> lidar_;
    std::shared_ptr<nodes::MotorNode> motor_;
    std::shared_ptr<nodes::ImuNode> imu_;
    std::shared_ptr<nodes::CameraNode> camera_;

    double uptime_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time last_time_;

    float front_limit_;

    float yaw_start_;
    float yaw_ref_;

    bool isCalibrated_;

    LineLoopState state_;
};



#endif //LINE_LOOP_HPP
