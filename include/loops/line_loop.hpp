//
// Created by student on 24.3.25.
//

#ifndef LINE_LOOP_HPP
#define LINE_LOOP_HPP
#include <memory>

#include "algorithms/pid.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/lidar_node.hpp"


class LineLoop : public rclcpp::Node {
public:
    // Constructor
    LineLoop(std::shared_ptr<nodes::LidarNode> lidar, std::shared_ptr<nodes::LineNode> line_sensors, std::shared_ptr<nodes::MotorNode> motor);
    // Destructor (default)
    ~LineLoop() override = default;
    void Restart();
private:
    algorithms::Pid pid_;
    void line_loop_timer_callback();

    std::shared_ptr<nodes::LineNode> line_sensors_;
    std::shared_ptr<nodes::LidarNode> lidar_;
    std::shared_ptr<nodes::MotorNode> motor_;

    double uptime_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time last_time_;
    bool isStarted_;

    float front_limit_;
};



#endif //LINE_LOOP_HPP
