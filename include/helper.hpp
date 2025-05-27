//
// Created by student on 3.3.25.
//

#ifndef HELPER_HPP
#define HELPER_HPP
#pragma once

#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>

static const int MAIN_LOOP_PERIOD_MS = 50;
static const int TICKS_PER_ROTATION = 576;
static const double WHEEL_RADIUS_METERS = 0.06517/2;
static const double WHEEL_BASE_METERS = 0.133;
static const double ENCODER_POLLING_RATE_MS = 1;

static const uint8_t MIN_MOTOR_SPEED = 127;
static const uint8_t MAX_MOTOR_SPEED = 145;
static const uint8_t MAX_TURNING_MOTOR_SPEED = 12;

static const float BASE_LINEAR_VELOCITY = 0.04;
static const float MAX_ANGULAR_VELOCITY = 0.05; // rad/s


static const float LINE_SENSOR_MAX_CALIBRATED_BLACK = 200;
static const int LINE_SENSORS_DEADZONE = 15;

// In meters
static const float MIN_FRONT_DISTANCE = 0.75f;
static const float FRONT_WALL_DISTANCE = 0.23f;
static const float WALL_DISTANCE = 0.22f;
static const float MIN_OPEN_SIDE_DISTANCE = 0.4f;
static const float DELTA_TO_TURN = 0.16f;

// In degrees
static const float MAX_YAW_ERROR = 15.f;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string line_sensors = "/bpc_prp_robot/line_sensors";
    const std::string lidar = "/bpc_prp_robot/lidar";
    const std::string imu = "/bpc_prp_robot/imu";
    const std::string camera = "/bpc_prp_robot/camera/compressed";
    const std::string lidar_avg = "/lidar_avg";
    const std::string detected_turn_type = "/detected_turn_type";
};

enum ArucoType {
    None = -1,
    Straight = 0,
    Left = 1,
    Right = 2,
    TreasureStraight = 10,
    TreasureLeft = 11,
    TreasureRight = 12,
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};

inline float mean(const std::vector<float>& vec) {
    if (vec.empty()) {
        // Handle empty vector case (avoid division by zero)
        //std::cerr << "Vector is empty, cannot calculate mean." << std::endl;
        return -1.0f;  // or some other error value
    }

    // Sum all elements in the vector using std::accumulate
    float sum = std::accumulate(vec.begin(), vec.end(), 0.0f);

    // Calculate mean
    float mean = sum / vec.size();
    return mean;
}

// Select 5-min values and calc their mean
inline float min_mean(std::vector<float>& vec)
{
    if (vec.empty()) {
        // Handle empty vector case (avoid division by zero)
        //std::cerr << "Vector is empty, cannot calculate mean." << std::endl;
        return -1.0f;  // or some other error value
    }

    // clamp the size
    size_t clampedCount = std::min(vec.size(), size_t(5));
    std::partial_sort(vec.begin(), vec.begin() + clampedCount, vec.end());

    // Create a new vector with the first 5 (smallest) values
    std::vector<float> fiveMinValues(vec.begin(), vec.begin() + clampedCount);

    // Pass to mean() function
    float avg = mean(fiveMinValues);
    return avg;
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

inline double roundUpTo90(double angle) {
    return std::round(angle / 90.0) * 90.0;
}

inline uint8_t convert_speed_to_command(float wheel_speed)
{
    int command = 127 + static_cast<int>(std::round(10 * wheel_speed));
    command = std::min(255, std::max(MIN_MOTOR_SPEED + 5, command));
    return static_cast<uint8_t>(command);
}

#endif //HELPER_HPP
