//
// Created by student on 14.4.25.
//

#include "../../include/nodes/imu_node.hpp"

#include "helper.hpp"

nodes::ImuNode::ImuNode() : Node("imuNode"), last_time_(this->now()) {
    this->get_logger().set_level(rclcpp::Logger::Level::Warn);
    planar_integrator_ = algorithms::PlanarImuIntegrator();
    planar_integrator_.reset();

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
             Topic::imu, 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
}

void nodes::ImuNode::setMode(const ImuNodeMode setMode) {
    mode = setMode;
}

nodes::ImuNodeMode nodes::ImuNode::getMode() {
    return mode;
}

float nodes::ImuNode::getIntegratedResults() {
    return planar_integrator_.getYaw();
}

void nodes::ImuNode::reset_imu() {
    planar_integrator_.reset();
    //mode = ImuNodeMode::CALIBRATE;
}

void nodes::ImuNode::calibrate(float gyro_z) {
    if (gyro_calibration_samples_.size() < 500)
    {
        gyro_calibration_samples_.push_back(gyro_z);
        planar_integrator_.setCalibration(gyro_calibration_samples_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully calibrated");
        mode = ImuNodeMode::INTEGRATE;
        integrate(gyro_z);
    }
}

void nodes::ImuNode::integrate(float gyro_z) {
    const double dt = (this->now() - last_time_).seconds();
    planar_integrator_.update(gyro_z, dt);
    RCLCPP_INFO(this->get_logger(), "Yaw: %.2f°", planar_integrator_.getYaw());
    last_time_ = this->now();
}

void nodes::ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
    float gyro_z = msg->angular_velocity.z;
    RCLCPP_DEBUG(this->get_logger(), "gyro_z: %f", gyro_z);
    switch (mode) {
        case ImuNodeMode::CALIBRATE:
        {
            calibrate(gyro_z);
        }
        break;

        case ImuNodeMode::INTEGRATE:
        default:
        {
            integrate(gyro_z);
        }
        break;
    }
}
