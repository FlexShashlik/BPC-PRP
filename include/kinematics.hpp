//
// Created by student on 10.3.25.
//

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP



class kinematics {
    static double angular_velocity_to_rad_per_s(double angVel);

public:
    // Calculate speed in meters per second
    static double calc_wheel_speed(double angVel);
    static double calc_delta_speed(double vL, double vR);
    static double calc_delta_angle(double vL, double vR);
    static void calc_pos(double dV, double dAngle);

    static double xPos;
    static double yPos;
    static double anglePos;
};



#endif //KINEMATICS_HPP
