//
// Created by student on 24.3.25.
//

#ifndef LINE_LOOP_HPP
#define LINE_LOOP_HPP
#include <memory>

#include "kinematics.hpp"
#include "algorithms/pid.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/lidar_node.hpp"

namespace nodes {
    class ImuNode;
}

// TurnStages to distinguish closed rotations and rotations inside X-sections where we need to detect turning point from the edge of the wall
enum class TurnStage {
    INIT,
    FORWARD_TO_EDGE,
    ROTATE,
    DONE
};

enum class LineLoopState {
    CALIBRATION,
    CORRIDOR_FOLLOWING,
    TURNING,
    AFTER_TURNING,
};

class LineLoop : public rclcpp::Node {
public:
    // Constructor
    LineLoop(std::shared_ptr<nodes::CameraNode> camera, std::shared_ptr<nodes::ImuNode> imu, std::shared_ptr<nodes::MotorNode> motor);
    // Destructor (default)
    ~LineLoop() override = default;
    void Restart();

    LineLoopState getState() const;

    void TurnLeft(TurnStage turn_stage = TurnStage::ROTATE);
    void TurnRight(TurnStage turn_stage = TurnStage::ROTATE);
    void Turn180(TurnStage turn_stage = TurnStage::ROTATE);

    float calculate_pid_angular_velocity(float left_dist, float right_dist, float dt);
    float calculate_left_wall_pid_angular_velocity(float left_dist, float dt);
    float calculate_right_wall_pid_angular_velocity(float right_dist, float dt);

private:
    algorithms::Pid pid_;
    void maze_loop(float dtMS);
    ArucoType getNextMove();
    bool doTurn(ArucoType nextMove);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    void on_lidar_msg(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    std::shared_ptr<nodes::MotorNode> motor_;
    std::shared_ptr<nodes::ImuNode> imu_;
    std::shared_ptr<nodes::CameraNode> camera_;

    algorithms::LidarFiltrResults lidar_results_;

    double uptime_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time last_time_;

    float front_limit_;

    float yaw_start_;
    float yaw_ref_;

    bool isCalibrated_;

    LineLoopState state_;

    // Do not use directly, use getNextMove() instead
    ArucoType nextMove_;

    float integral_;
    float previous_error_;
    float kp_, ki_, kd_;

    float base_linear_velocity_;
    algorithms::Kinematics kinematics_;

    TurnStage turn_stage_ = TurnStage::INIT;
    float initial_front_ = 0.0f;
};



#endif //LINE_LOOP_HPP
