//
// Created by student on 17.3.25.
//

#ifndef LINE_HPP
#define LINE_HPP
#include <rclcpp/node.hpp>
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace nodes {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineNode : public rclcpp::Node{
    public:

        LineNode();

        ~LineNode() override = default;

        // relative pose to line in meters
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

        /*static*/ DiscreteLinePose estimate_descrete_line_pose(uint16_t left, uint16_t right);
    private:

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);

        //float estimate_continuous_line_pose(float left_value, float right_value);

        uint16_t line_sensor_left_ = 0;
        uint16_t line_sensor_right_ = 0;
    };
}


#endif //LINE_HPP
