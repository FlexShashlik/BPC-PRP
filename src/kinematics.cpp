//
// Created by student on 10.3.25.
//

#include "../include/kinematics.hpp"

#include <math.h>

#include "helper.hpp"

double kinematics::xPos = 0.0;
double kinematics::yPos = 0.0;
double kinematics::anglePos = 0.0;

double kinematics::angular_velocity_to_rad_per_s(double angVel) {
    return angVel * 2 * M_PI;
}

// Takes angVel in rotations per seconds
double kinematics::calc_wheel_speed(double angVel) {
    return kinematics::angular_velocity_to_rad_per_s(angVel) * WHEEL_RADIUS_METERS;
}

double kinematics::calc_delta_speed(double vL, double vR) {
    return (vL + vR) / 2.;
}

double kinematics::calc_delta_angle(double vL, double vR) {
    return (vR - vL) / WHEEL_BASE_METERS;
}

// Takes deltaAngle in rad calculated from calc_delta_angle
void kinematics::calc_pos(double dV, double dAngle) {
    xPos = xPos + dV * cos(dAngle/2.);
    yPos = yPos + dV * sin(dAngle/2.);
    anglePos = anglePos + dAngle;
}