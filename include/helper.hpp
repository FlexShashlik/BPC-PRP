//
// Created by student on 3.3.25.
//

#ifndef HELPER_HPP
#define HELPER_HPP
#pragma once

#include <iostream>
#include <vector>
#include <numeric>

static const int MAIN_LOOP_PERIOD_MS = 50;
static const int TICKS_PER_ROTATION = 576;
static const double WHEEL_RADIUS_METERS = 0.06517/2;
static const double WHEEL_BASE_METERS = 0.133;
static const double LOOP_POLLING_RATE_MS = 10;
static const double ENCODER_POLLING_RATE_MS = 1;

static const uint8_t MIN_MOTOR_SPEED = 127;
static const uint8_t MAX_MOTOR_SPEED = 138;
static const uint8_t MAX_TURNING_MOTOR_SPEED = 5;


static const float LINE_SENSOR_MAX_CALIBRATED_BLACK = 200;
static const int LINE_SENSORS_DEADZONE = 15;

// In meters
static const float MIN_FRONT_DISTANCE = 0.21f;
static const float MIN_OPEN_SIDE_DISTANCE = 0.35f;

// In degrees
static const float MAX_YAW_ERROR = 7.f;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string line_sensors = "/bpc_prp_robot/line_sensors";
    const std::string lidar = "/bpc_prp_robot/lidar";
    const std::string imu = "/bpc_prp_robot/imu";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};

inline float mean(const std::vector<float>& vec) {
    if (vec.empty()) {
        // Handle empty vector case (avoid division by zero)
        std::cerr << "Vector is empty, cannot calculate mean." << std::endl;
        return -1.0f;  // or some other error value
    }

    // Sum all elements in the vector using std::accumulate
    float sum = std::accumulate(vec.begin(), vec.end(), 0.0f);

    // Calculate mean
    float mean = sum / vec.size();
    return mean;
}

constexpr double deg2rad(const double deg) {
    return deg * M_PI / 180.0;
}

constexpr double rad2deg(const double rad) {
    return rad * 180.0 / M_PI;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


#endif //HELPER_HPP
