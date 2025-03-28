//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "helper.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

LineLoop::LineLoop (std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor) : Node(
    "lineLoopNode"), pid_(10, 0.01, 0.05), last_time_(this->now()) {
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(LOOP_POLLING_RATE_MS)),
        std::bind(&LineLoop::line_loop_timer_callback, this));

    line_sensors_ = line_sensors;
    motor_ = motor;
}

void LineLoop::line_loop_timer_callback() {
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

    // PID
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
    // PID END
}
