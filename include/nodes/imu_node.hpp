#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"
#include "loops/line_loop.hpp"


#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        float getIntegratedResults();

        // Reset the class
        void reset_imu();

    private:

        void calibrate(float gyro_z);
        void integrate(float gyro_z);

        ImuNodeMode mode = ImuNodeMode::CALIBRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);

        // Start time for dt calculation
        rclcpp::Time last_time_;
    };
}
#endif