//
// Created by student on 3.3.25.
//

#ifndef HELPER_HPP
#define HELPER_HPP
#pragma once

#include <iostream>

static const int MAIN_LOOP_PERIOD_MS = 50;
static const int TICKS_PER_ROTATION = 576;
static const double WHEEL_RADIUS_METERS = 0.06517/2;
static const double WHEEL_BASE_METERS = 0.133;
static const double LOOP_POLLING_RATE_MS = 30;
static const double ENCODER_POLLING_RATE_MS = 10;

static const uint8_t MIN_MOTOR_SPEED = 127;
static const uint8_t MAX_MOTOR_SPEED = 140;

static const float LINE_SENSOR_MAX_CALIBRATED_BLACK = 700;
static const int LINE_SENSORS_DEADZONE = 15;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string line_sensors = "/bpc_prp_robot/line_sensors";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};

#endif //HELPER_HPP
