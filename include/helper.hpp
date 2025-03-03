//
// Created by student on 3.3.25.
//

#ifndef HELPER_HPP
#define HELPER_HPP
#pragma once

#include <iostream>

static const int MAIN_LOOP_PERIOD_MS = 50;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};

#endif //HELPER_HPP
