//
// Created by student on 14.4.25.
//

#ifndef PLANAR_IMU_INTEGRATOR_HPP
#define PLANAR_IMU_INTEGRATOR_HPP
#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        // Call this regularly to integrate gyro_z over time
        void update(float gyro_z, double dt);

        // Calibrate the gyroscope by computing average from static samples in radians
        void setCalibration(std::vector<float> gyro);

        // Returns the current estimated yaw in degrees
        [[nodiscard]] float getYaw() const;

        // Reset orientation and calibration
        void reset();

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}

#endif //PLANAR_IMU_INTEGRATOR_HPP
