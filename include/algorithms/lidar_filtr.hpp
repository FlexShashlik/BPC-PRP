//
// Created by student on 7.4.25.
//

#ifndef LIDAR_FILTR_HPP
#define LIDAR_FILTR_HPP



#include <cmath>
#include <vector>
#include <numeric>

#include "helper.hpp"

namespace algorithms {
    // Structure to store filtered average distances in key directions
    struct LidarFiltrResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFiltr {
    public:
        LidarFiltr() = default;

        static LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = deg2rad(10);

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                // TODO: Skip invalid (infinite) readings
                if (points[i] == INFINITY || points[i] == -INFINITY || points[i] == NAN) {
                    continue;
                }

                // TODO: Sort the value into the correct directional bin based on angle
                if (angle >= M_PI-angle_range || angle < -M_PI+angle_range) {
                    front.push_back(points[i]);
                } else if (angle >= -M_PI/2-angle_range && angle < -M_PI/2+angle_range) {
                    left.push_back(points[i]);
                } else if (angle >= -angle_range && angle < angle_range) {
                    back.push_back(points[i]);
                } else if (angle >= M_PI/2-angle_range && angle < M_PI/2+angle_range) {
                    right.push_back(points[i]);
                }
            }

            // TODO: Return the average of each sector (basic mean filter)
            return LidarFiltrResults{
                .front = mean(front),
                .back = mean(back),
                .left = mean(left),
                .right = mean(right),
            };
        }
    };
}



#endif //LIDAR_FILTR_HPP
