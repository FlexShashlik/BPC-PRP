#include <rclcpp/rclcpp.hpp>
#include "SineWave.hpp"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "loops/line_loop.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/lidar_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto io = std::make_shared<nodes::IoNode>();
    executor->add_node(io);

    auto motor = std::make_shared<nodes::MotorNode>();
    executor->add_node(motor);

    auto lidar_node = std::make_shared<nodes::LidarNode>();
    executor->add_node(lidar_node);

    auto imu = std::make_shared<nodes::ImuNode>();
    executor->add_node(imu);

    auto camera = std::make_shared<nodes::CameraNode>();
    executor->add_node(camera);

    auto line_loop = std::make_shared<LineLoop>(camera, imu, lidar_node, motor);
    executor->add_node(line_loop);

    auto executor_thread = std::thread([& executor](){executor->spin();});
    while (rclcpp::ok())
    {
        motor->start();
        motor->go(140,140);
        if (imu->getMode() == nodes::ImuNodeMode::CALIBRATE)
        {
            continue;
        }

        switch (io->get_button_pressed()) {
            case 0:
            {
                motor->stop();
            }
            break;
            case 1:
            {
                motor->start();
                line_loop->Restart();
            }
            break;
            default:
                break;
        }
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
