//
// Created by student on 24.3.25.
//

#ifndef LINE_LOOP_HPP
#define LINE_LOOP_HPP
#include <memory>

#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"


class line_loop {
public:
    static void LineLoop(std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor);
};



#endif //LINE_LOOP_HPP
