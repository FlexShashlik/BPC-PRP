//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "helper.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

LineLoop::LineLoop (std::shared_ptr<nodes::ImuNode> imu, std::shared_ptr<nodes::LidarNode> lidar, std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor) : Node(
    "lineLoopNode"), pid_(30, 0.03, 0), last_time_(this->now()) {
    this->get_logger().set_level(rclcpp::Logger::Level::Info);
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(LOOP_POLLING_RATE_MS)),
        std::bind(&LineLoop::line_loop_timer_callback, this));

    lidar_ = lidar;
    line_sensors_ = line_sensors;
    motor_ = motor;

    front_limit_ = MIN_FRONT_DISTANCE;
    imu_ = imu;

    isCalibrated_ = false;
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
    // BANG-BANG
    {
        /*switch (line_sensors_->get_discrete_line_pose()) {
            case nodes::DiscreteLinePose::LineOnRight:
                motor_->go_left(0);
                break;
            case nodes::DiscreteLinePose::LineOnLeft:
                motor_->go_right(0);
                break;
            case nodes::DiscreteLinePose::LineNone:
            default:
                motor_->go_forward(0);
                break;
        }*/
    }
    // BANG-BANG END

    // PID using encoders
    {
        /*
        int inputPid = 0;
        switch (line_sensors_->get_discrete_line_pose()) {
            case nodes::DiscreteLinePose::LineOnRight:
                inputPid = 1;
                break;
            case nodes::DiscreteLinePose::LineOnLeft:
                inputPid = -1;
                break;
            case nodes::DiscreteLinePose::LineNone:
            default:
                inputPid = 0;
                break;
        }
        float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

        uint8_t l = MAX_MOTOR_SPEED + outputPid;
        uint8_t r = MAX_MOTOR_SPEED - outputPid;

        uint8_t outL, outR;
        outL = std::clamp(l, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        outR = std::clamp(r, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

        motor_->go(outL, outR);
        */
    }
    // END PID using encoders

    // Lidar only PID
    {/*
    algorithms::LidarFiltrResults results = lidar_->GetLidarFiltrResults();
    if (results.front > front_limit_ && results.right < MIN_FRONT_DISTANCE && results.left < MIN_FRONT_DISTANCE) {
        front_limit_ = MIN_FRONT_DISTANCE;
        // PID using LIDAR
        float inputPid = results.right - results.left;
        float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

        uint8_t l = MAX_MOTOR_SPEED + outputPid;
        uint8_t r = MAX_MOTOR_SPEED - outputPid;

        uint8_t outL, outR;
        outL = std::clamp(l, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        outR = std::clamp(r, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

        motor_->go(outL, outR);
        //RCLCPP_INFO(this->get_logger(), "PID :: outL:%u outR:%u PID:%f", outL, outR, outputPid);
        // END PID using LIDAR
    }
    else
    {
        front_limit_ = 2.f;
        if (results.front < MIN_FRONT_DISTANCE) {
            if (results.right > results.left) {
                motor_->go_right();
            }
            else if (results.right < results.left) {
                motor_->go_left();
            }
        }
        else {
            motor_->go_forward();
        }*/
    }
    // END Lidar only PID

    // STATE MACHINE
    algorithms::LidarFiltrResults results;
    switch (state_) {
        case LineLoopState::CALIBRATION:
        // Wait until enough samples are collected
        // Once done, switch to CORRIDOR_FOLLOWING
        break;

        case LineLoopState::CORRIDOR_FOLLOWING:
        // Keep centered using P/PID based on side distances
        // If front is blocked and one side is open → switch to TURNING
        {
            results = lidar_->GetLidarFiltrResults();
            if (results.front > front_limit_) {
                front_limit_ = MIN_FRONT_DISTANCE;
                // PID using LIDAR
                float inputPid = results.right - results.left;
                float outputPid = pid_.step(inputPid, LOOP_POLLING_RATE_MS);

                uint8_t l = MAX_MOTOR_SPEED + outputPid;
                uint8_t r = MAX_MOTOR_SPEED - outputPid;

                uint8_t outL, outR;
                outL = std::clamp(l, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
                outR = std::clamp(r, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

                motor_->go(outL, outR);
                // END PID using LIDAR
            }
            else
            {
                state_ = LineLoopState::TURNING;
                yaw_start_ = imu_->getIntegratedResults();
                RCLCPP_INFO(this->get_logger(), "State TURNING from: %.2f°", yaw_start_);

                if (results.right > MIN_OPEN_SIDE_DISTANCE)
                {
                    yaw_ref_ = rad2deg(deg2rad(yaw_start_) - (M_PI / 2));
                    //yaw_ref_ = yaw_start_ + 90;
                    RCLCPP_INFO(this->get_logger(), "TURNING RIGHT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
                else if (results.left > MIN_OPEN_SIDE_DISTANCE)
                {
                    yaw_ref_ = rad2deg(deg2rad(yaw_start_) + (M_PI / 2));
                    //yaw_ref_ = yaw_start_ - 90;
                    RCLCPP_INFO(this->get_logger(), "TURNING LEFT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
                else
                {
                    yaw_ref_ = yaw_start_ + 180;
                    RCLCPP_INFO(this->get_logger(), "TURNING 180 from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
                }
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
                state_ = LineLoopState::CORRIDOR_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "State CORRIDOR FOLLOWING yaw_start: %.2f°, yaw_ref: %.2f°, current_yaw: %.2f°", yaw_start_, yaw_ref_, current_yaw);
            }
            else
            {
                //float outputPid = pid_.step(yaw_error, LOOP_POLLING_RATE_MS);
                float outputPid = 0.1 * yaw_error;
                uint8_t l = MAX_TURNING_MOTOR_SPEED - outputPid;
                uint8_t r = MAX_TURNING_MOTOR_SPEED + outputPid;

                uint8_t outL, outR;
                outL = std::clamp(l, MIN_MOTOR_SPEED, MAX_TURNING_MOTOR_SPEED);
                outR = std::clamp(r, MIN_MOTOR_SPEED, MAX_TURNING_MOTOR_SPEED);

                motor_->go(outL, outR);
                RCLCPP_INFO(this->get_logger(), "PID :: outL:%u outR:%u PID:%f", outL, outR, outputPid);
            }
        }
        break;
    }
    // END State machine
}
