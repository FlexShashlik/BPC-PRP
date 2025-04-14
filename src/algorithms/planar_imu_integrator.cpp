//
// Created by student on 14.4.25.
//

#include "algorithms/planar_imu_integrator.hpp"

#include "helper.hpp"

void algorithms::PlanarImuIntegrator::update(float gyro_z, double dt) {
    theta_ += (gyro_z - gyro_offset_) * dt;
}

void algorithms::PlanarImuIntegrator::setCalibration(std::vector<float> gyro) {
    gyro_offset_ = mean(gyro);
}

// Angle (degrees)
float algorithms::PlanarImuIntegrator::getYaw() const {
    return rad2deg(theta_);
}

void algorithms::PlanarImuIntegrator::reset() {
    theta_ = 0;
    gyro_offset_ = 0;
}
