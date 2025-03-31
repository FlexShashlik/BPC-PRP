//
// Created by student on 17.3.25.
//

#include "nodes/line_node.hpp"

#include "../../include/nodes/line_node.hpp"

#include "helper.hpp"

#include <algorithm>

namespace nodes {
    LineNode::LineNode() : Node("lineNode") {
        line_sensor_left_ = 0;
        line_sensor_right_ = 0;
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
             Topic::line_sensors, 1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
    }

    void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            line_sensor_left_ = msg->data[0];
            line_sensor_right_ = msg->data[1];
            RCLCPP_INFO(this->get_logger(), "Line_sensor_left: %u", this->line_sensor_left_);
            RCLCPP_INFO(this->get_logger(), "Line_sensor_right: %u", this->line_sensor_right_);

            discrete_line_pose_ = estimate_descrete_line_pose(line_sensor_left_, line_sensor_right_);
            float analog_pose = get_continuous_line_pose();
            RCLCPP_DEBUG(this->get_logger(), "Analog pose: %f mm", analog_pose);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty UInt32MultiArray message.");
        }
    }

    DiscreteLinePose LineNode::estimate_descrete_line_pose(uint16_t left, uint16_t right) {
        // positive - turn to right
        // negative = turn to left
        int dir = right - left;

        // Both on white
        if (dir >= -LINE_SENSORS_DEADZONE  && dir <= LINE_SENSORS_DEADZONE) {
            RCLCPP_INFO(this->get_logger(), "White on both");
            return DiscreteLinePose::LineNone;
        }
        else if (dir > LINE_SENSORS_DEADZONE) {
            RCLCPP_INFO(this->get_logger(), "Line on right");
            return DiscreteLinePose::LineOnRight;
        }
        else if (dir < -LINE_SENSORS_DEADZONE) {
            RCLCPP_INFO(this->get_logger(), "Line on left");
            return DiscreteLinePose::LineOnLeft;
        }
        else {
            return DiscreteLinePose::LineBoth;
        }
    }

    float LineNode::get_continuous_line_pose() const {
        const uint16_t min = 30;
        const int krok = 10;
        const uint16_t max = 400;
        const float r = 2; //mm radius snimace

        auto l_clamped = std::clamp(line_sensor_left_,min,max);
        auto l_discr = l_clamped - (l_clamped % krok) - min; // 0 - (max-min)
        auto l_norm = (r/(max-min) * l_discr); // mm

        auto r_clamped = std::clamp(line_sensor_right_,min,max);
        auto r_discr = r_clamped - (r_clamped % krok) - min; // 0 - (max-min)
        auto r_norm = (r/(max-min) * r_discr); // mm

        return r_norm - l_norm;


        //int dir = right - left;
        //line_sensor_left_
        //line_sensor_right_
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() const {
        return discrete_line_pose_;
    }
}
