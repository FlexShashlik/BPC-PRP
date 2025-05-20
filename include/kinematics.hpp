//
// Created by student on 10.3.25.
//

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP



#include <cmath>

namespace algorithms
{

    struct RobotSpeed
    {
        float v; // linear velocity in m/s
        float w; // angular velocity in rad/s

        // Constructor for easy initialization
        RobotSpeed(float linear = 0.0f, float angular = 0.0f) : v(linear), w(angular) {}
    };

    struct WheelSpeed
    {
        float l; // left wheel speed in rad/s
        float r; // right wheel speed in rad/s

        // Constructor for easy initialization
        WheelSpeed(float left = 0.0f, float right = 0.0f) : l(left), r(right) {}
    };

    struct Encoders
    {
        int l; // left encoder ticks
        int r; // right encoder ticks

        // Constructor for easy initialization
        Encoders(int left = 0, int right = 0) : l(left), r(right) {}
    };

    struct Coordinates
    {
        float x;     // x position in meters
        float y;     // y position in meters
        float theta; // orientation in radians

        // Constructor for easy initialization
        Coordinates(float x_pos = 0.0f, float y_pos = 0.0f, float angle = 0.0f)
            : x(x_pos), y(y_pos), theta(angle) {}
    };

    class Kinematics
    {
    private:
        double wheel_radius_;     // radius of the wheels in meters
        double wheel_base_;       // distance between wheels in meters
        int pulses_per_rotation_; // encoder ticks per full wheel rotation

    public:
        /**
         * Constructor for the Kinematics class
         *
         * @param wheel_radius Radius of the wheels in meters
         * @param wheel_base Distance between wheels in meters
         * @param pulses_per_rotation Number of encoder pulses per complete wheel rotation
         */
        Kinematics(double wheel_radius, double wheel_base, int pulses_per_rotation)
            : wheel_radius_(wheel_radius),
              wheel_base_(wheel_base),
              pulses_per_rotation_(pulses_per_rotation) {}

        /**
         * Forward kinematics: Calculate robot velocity from wheel speeds
         *
         * @param wheel_speeds Speeds of the left and right wheels (rad/s)
         * @return Robot linear and angular velocity
         */
        RobotSpeed forward(const WheelSpeed &wheel_speeds) const
        {
            RobotSpeed robot_speed;

            // Calculate linear velocity (average of both wheels' contribution)
            robot_speed.v = wheel_radius_ * (wheel_speeds.r + wheel_speeds.l) / 2.0f;

            // Calculate angular velocity (difference between wheels divided by wheel base)
            robot_speed.w = wheel_radius_ * (wheel_speeds.r - wheel_speeds.l) / wheel_base_;

            return robot_speed;
        }

        /**
         * Inverse kinematics: Calculate wheel speeds from robot velocity
         *
         * @param robot_speed Robot linear and angular velocity
         * @return Required speeds of the left and right wheels
         */
        WheelSpeed inverse(const RobotSpeed &robot_speed) const
        {
            WheelSpeed wheel_speeds;

            // Calculate wheel speeds based on robot linear and angular velocity
            wheel_speeds.l = (robot_speed.v - (wheel_base_ / 2.0f) * robot_speed.w) / wheel_radius_;
            wheel_speeds.r = (robot_speed.v + (wheel_base_ / 2.0f) * robot_speed.w) / wheel_radius_;

            return wheel_speeds;
        }

        /**
         * Forward kinematics for encoder readings
         *
         * @param encoders Encoder ticks for left and right wheels
         * @return Robot displacement in Cartesian coordinates
         */
        Coordinates forward(const Encoders &encoders) const
        {
            Coordinates displacement;

            // Convert encoder ticks to wheel rotations
            float left_rotation = static_cast<float>(encoders.l) / pulses_per_rotation_;
            float right_rotation = static_cast<float>(encoders.r) / pulses_per_rotation_;

            // Convert rotations to distance traveled by each wheel
            float left_distance = left_rotation * 2.0f * M_PI * wheel_radius_;
            float right_distance = right_rotation * 2.0f * M_PI * wheel_radius_;

            // Calculate forward distance and rotation
            float distance = (left_distance + right_distance) / 2.0f;
            float rotation = (right_distance - left_distance) / wheel_base_;

            // Simple straight-line approximation if rotation is very small
            if (std::abs(rotation) < 1e-6)
            {
                displacement.x = distance;
                displacement.y = 0;
                displacement.theta = 0;
            }
            else
            {
                // Calculate displacement using arc equations
                float radius = distance / rotation;
                displacement.x = radius * std::sin(rotation);
                displacement.y = radius * (1.0f - std::cos(rotation));
                displacement.theta = rotation;
            }

            return displacement;
        }
    };

} // namespace algorithms



#endif //KINEMATICS_HPP
