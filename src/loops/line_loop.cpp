//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "helper.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

LineLoop::LineLoop (std::shared_ptr<nodes::CameraNode> camera, std::shared_ptr<nodes::ImuNode> imu, std::shared_ptr<nodes::LidarNode> lidar, std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor) : Node(
    "lineLoopNode"), pid_(12, 0, 1), last_time_(this->now()) {
    this->get_logger().set_level(rclcpp::Logger::Level::Warn);
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(LOOP_POLLING_RATE_MS)),
        std::bind(&LineLoop::line_loop_timer_callback, this));

    lidar_ = lidar;
    line_sensors_ = line_sensors;
    motor_ = motor;

    front_limit_ = MIN_FRONT_DISTANCE;
    imu_ = imu;
    camera_ = camera;

    isCalibrated_ = false;

    nextMove_ = ArucoType::None;
}

void LineLoop::Restart() {
    pid_.reset();
    last_time_ = this->now();
}

LineLoopState LineLoop::getState() const {
    return state_;
}

void LineLoop::line_loop_timer_callback() {
    if (imu_->getMode() == nodes::ImuNodeMode::CALIBRATE)
    {
        state_ = LineLoopState::CALIBRATION;
        return;
    }
    if (!isCalibrated_ && imu_->getMode() != nodes::ImuNodeMode::CALIBRATE)
    {
        isCalibrated_ = true;
        state_ = LineLoopState::CORRIDOR_FOLLOWING;
        RCLCPP_INFO(this->get_logger(), "State CORRIDOR_FOLLOWING");
    }

    // STATE MACHINE_EXPERIMENTAL
    algorithms::LidarFiltrResults results;

    switch (state_)
    {
        case LineLoopState::CALIBRATION:
            // Wait until enough samples are collected
            // Once done, switch to CORRIDOR_FOLLOWING
        break;

        case LineLoopState::CORRIDOR_FOLLOWING:
        // Keep centered using P/PID based on side distances
        // If front is blocked and one side is open → switch to TURNING
        {
            results = lidar_->GetLidarFiltrResults();
            if (results.front == -1 || results.right == -1 || results.left == -1 || results.front_right == -1 || results.front_left == -1)
            {
                RCLCPP_INFO(this->get_logger(), "Empty, continue..");

                if (results.left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty left, too close probably, substitute..");
                    results.left = 0;
                }

                if (results.right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty right, too close probably, substitute..");
                    results.right = 0;
                }

                if (results.front_right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_right, too close probably, substitute..");
                    results.front_right = 0;
                }

                if (results.front_left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_left, too close probably, substitute..");
                    results.front_left = 0;
                }

                if (results.front == -1)
                    return;
            }

            if (results.front > WALL_DISTANCE)
            {
                if (results.right < MIN_OPEN_SIDE_DISTANCE && results.left < MIN_OPEN_SIDE_DISTANCE)
                {
                    float inputPid = 0.22f - results.left;
                    float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                    uint8_t l = MAX_MOTOR_SPEED + outputPid;
                    uint8_t r = MAX_MOTOR_SPEED - outputPid;

                    uint8_t outL, outR;
                    outL = std::clamp(l, (uint8_t)120, MAX_MOTOR_SPEED);
                    outR = std::clamp(r, (uint8_t)120, MAX_MOTOR_SPEED);

                    motor_->go(outL, outR);
                }
                else if (results.right > MIN_OPEN_SIDE_DISTANCE && results.left < MIN_OPEN_SIDE_DISTANCE)
                {
                    if (results.front < MIN_FRONT_DISTANCE || !doTurn(getNextMove()))
                    {
                        float inputPid = 0.22f - results.left;
                        float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                        uint8_t l = MAX_MOTOR_SPEED + outputPid;
                        uint8_t r = MAX_MOTOR_SPEED - outputPid;

                        uint8_t outL, outR;
                        outL = std::clamp(l, (uint8_t)120, MAX_MOTOR_SPEED);
                        outR = std::clamp(r, (uint8_t)120, MAX_MOTOR_SPEED);

                        motor_->go(outL, outR);
                    }
                }
                else if (results.right < MIN_OPEN_SIDE_DISTANCE && results.left > MIN_OPEN_SIDE_DISTANCE)
                {
                    if (results.front < MIN_FRONT_DISTANCE || !doTurn(getNextMove()))
                    {
                        float inputPid = results.right - 0.22f;
                        float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                        uint8_t l = MAX_MOTOR_SPEED + outputPid;
                        uint8_t r = MAX_MOTOR_SPEED - outputPid;

                        uint8_t outL, outR;
                        outL = std::clamp(l, (uint8_t)120, MAX_MOTOR_SPEED);
                        outR = std::clamp(r, (uint8_t)120, MAX_MOTOR_SPEED);

                        motor_->go(outL, outR);
                    }
                }
                else
                {
                    // X-section
                    // If less than min distance to be a real X-section (can be just going to T-section)
                    //if (/*results.front < MIN_FRONT_DISTANCE || !doTurn(getNextMove())*/)
                    {
                        motor_->go(135, 135);
                    }
                }
            }
            else // Closed turns (no way forward)
            {
                if (nextMove_ == ArucoType::Straight || nextMove_ == ArucoType::TreasureStraight)
                    nextMove_ = ArucoType::None;

                yaw_start_ = imu_->getIntegratedResults();
                if (results.left < MIN_OPEN_SIDE_DISTANCE && results.right < MIN_OPEN_SIDE_DISTANCE)
                {
                    // Dead end
                    // turning 180
                    yaw_ref_ = yaw_start_ + 180;
                    state_ = LineLoopState::TURNING;
                    RCLCPP_WARN(this->get_logger(), "TURNING 180 from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
                else if (results.left > MIN_OPEN_SIDE_DISTANCE && results.right < MIN_OPEN_SIDE_DISTANCE)
                {
                    // turning to right
                    yaw_ref_ = rad2deg(deg2rad(yaw_start_) + (M_PI / 2));
                    state_ = LineLoopState::TURNING;
                    RCLCPP_WARN(this->get_logger(), "TURNING RIGHT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
                else if (results.right > MIN_OPEN_SIDE_DISTANCE && results.left < MIN_OPEN_SIDE_DISTANCE)
                {
                    // turning to left
                    yaw_ref_ = rad2deg(deg2rad(yaw_start_) - (M_PI / 2));
                    state_ = LineLoopState::TURNING;
                    RCLCPP_WARN(this->get_logger(), "TURNING LEFT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
                else
                {
                    // T-turn
                    if (!doTurn(getNextMove()))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Can't go straight in a T-turn!");
                        motor_->go(127, 127);
                        motor_->stop();
                    }
                }
            }
        }
        break;

        case LineLoopState::AFTER_TURNING:
        {
            results = lidar_->GetLidarFiltrResults();
            if (results.front == -1 || results.right == -1 || results.left == -1 || results.front_right == -1 || results.front_left == -1)
            {
                RCLCPP_INFO(this->get_logger(), "Empty, continue..");

                if (results.left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty left, too close probably, substitute..");
                    results.left = 0;
                }

                if (results.right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty right, too close probably, substitute..");
                    results.right = 0;
                }

                if (results.front_right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_right, too close probably, substitute..");
                    results.front_right = 0;
                }

                if (results.front_left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_left, too close probably, substitute..");
                    results.front_left = 0;
                }

                if (results.front == -1)
                    return;
            }

            if (results.right < MIN_OPEN_SIDE_DISTANCE && results.left < MIN_OPEN_SIDE_DISTANCE)
            {
                state_ = LineLoopState::CORRIDOR_FOLLOWING;
                RCLCPP_WARN(this->get_logger(), "State CORRIDOR FOLLOWING");
            }
            else if (results.right > MIN_OPEN_SIDE_DISTANCE && results.left < MIN_OPEN_SIDE_DISTANCE)
            {
                float inputPid = 0.22f - results.left;
                float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                uint8_t l = MAX_MOTOR_SPEED + outputPid;
                uint8_t r = MAX_MOTOR_SPEED - outputPid;

                uint8_t outL, outR;
                outL = std::clamp(l, (uint8_t)120, MAX_MOTOR_SPEED);
                outR = std::clamp(r, (uint8_t)120, MAX_MOTOR_SPEED);

                motor_->go(outL, outR);
            }
            else if (results.right < MIN_OPEN_SIDE_DISTANCE && results.left > MIN_OPEN_SIDE_DISTANCE)
            {
                float inputPid = results.right - 0.22f;
                float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                uint8_t l = MAX_MOTOR_SPEED + outputPid;
                uint8_t r = MAX_MOTOR_SPEED - outputPid;

                uint8_t outL, outR;
                outL = std::clamp(l, (uint8_t)120, MAX_MOTOR_SPEED);
                outR = std::clamp(r, (uint8_t)120, MAX_MOTOR_SPEED);

                motor_->go(outL, outR);
            }
            else
            {
                motor_->go(135, 135);
            }
        }
        break;

        case LineLoopState::TURNING:
            // Use IMU to track rotation
            // Rotate until yaw changes by ±90°
            // Then return to CORRIDOR_FOLLOWING
        {
            float current_yaw = imu_->getIntegratedResults();
            float yaw_error = yaw_ref_ - current_yaw;

            if (std::abs(yaw_error) < MAX_YAW_ERROR)
            {
                state_ = LineLoopState::AFTER_TURNING;
                RCLCPP_WARN(this->get_logger(), "State AFTER TURNING yaw_start: %.2f°, yaw_ref: %.2f°, current_yaw: %.2f°", yaw_start_, yaw_ref_, current_yaw);

                if (nextMove_ == Right || nextMove_ == Left || nextMove_ == TreasureRight || nextMove_ == TreasureLeft)
                    nextMove_ = None;
            }
            else
            {
                uint8_t l = 127 - (MAX_TURNING_MOTOR_SPEED * sgn(yaw_error));
                uint8_t r = 127 + (MAX_TURNING_MOTOR_SPEED * sgn(yaw_error));

                motor_->go(l, r);
                RCLCPP_INFO(this->get_logger(), "PID :: outL:%u outR:%u PID:%f", l, r);
            }
        }
        break;
    }

    // END State machine
}

ArucoType LineLoop::getNextMove()
{
    if (nextMove_ != ArucoType::None)
        return nextMove_;

    nextMove_ = camera_->GetNextMove();
    RCLCPP_WARN(this->get_logger(), "Next move: %i", (int)nextMove_);
    return nextMove_;
}

bool LineLoop::doTurn(const ArucoType nextMove)
{
    algorithms::LidarFiltrResults results = lidar_->GetLidarFiltrResults();
    yaw_start_ = imu_->getIntegratedResults();

    switch (nextMove)
    {
        case ArucoType::TreasureLeft:
        case ArucoType::Left:
        {
            float lastPos = results.front;
            float front = lidar_->GetLidarFiltrResults().front;

            while(front > lastPos-DELTA_TO_TURN && front > WALL_DISTANCE)
            {
                motor_->go(138, 138);
                front = lidar_->GetLidarFiltrResults().front;
            }

            yaw_ref_ = rad2deg(deg2rad(yaw_start_) + (M_PI / 2));
            RCLCPP_WARN(this->get_logger(), "doTurn LEFT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
            state_ = LineLoopState::TURNING;
            return true;
        }
        break;

        case ArucoType::TreasureRight:
        case ArucoType::Right:
        {
            float lastPos = results.front;
            float front = lidar_->GetLidarFiltrResults().front;

            while(front > lastPos-DELTA_TO_TURN && front > WALL_DISTANCE)
            {
                motor_->go(138, 138);
                front = lidar_->GetLidarFiltrResults().front;
            }

            motor_->go(127, 127);

            yaw_ref_ = rad2deg(deg2rad(yaw_start_) - (M_PI / 2));
            RCLCPP_WARN(this->get_logger(), "doTurn RIGHT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
            state_ = LineLoopState::TURNING;
            return true;
        }
        break;

        default:
        case ArucoType::TreasureStraight:
        case ArucoType::Straight:
        {
            RCLCPP_WARN(this->get_logger(), "doTurn Straight");
            return false;
        }
        break;
    }
}
