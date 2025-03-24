//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

void line_loop::LineLoop (std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor) {
    switch (line_sensors->get_discrete_line_pose()) {
        case nodes::DiscreteLinePose::LineOnRight:
            motor->go_left(0);
        break;
        case nodes::DiscreteLinePose::LineOnLeft:
            motor->go_right(0);
        break;
        case nodes::DiscreteLinePose::LineNone:
            default:
                motor->go_forward(0);
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
