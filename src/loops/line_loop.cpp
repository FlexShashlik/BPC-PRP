//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "helper.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

LineLoop::LineLoop (std::shared_ptr<nodes::LidarNode> lidar, std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor) : Node(
    "lineLoopNode"), pid_(40, 0.03, 0), last_time_(this->now()) {
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(LOOP_POLLING_RATE_MS)),
        std::bind(&LineLoop::line_loop_timer_callback, this));

    lidar_ = lidar;
    line_sensors_ = line_sensors;
    motor_ = motor;
    isStarted_ = false;

    front_limit_ = MIN_FRONT_DISTANCE;
}

void LineLoop::Restart() {
    pid_.reset();
    last_time_ = this->now();
    isStarted_ = true;
}

void LineLoop::line_loop_timer_callback() {
    if (!isStarted_)
        return;
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
        }
    }
}
