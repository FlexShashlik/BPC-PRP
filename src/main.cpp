#include <rclcpp/rclcpp.hpp>
#include "SineWave.hpp"
#include "nodes/io_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"
#include "loops/line_loop.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto io = std::make_shared<nodes::IoNode>();
    executor->add_node(io);

    auto motor = std::make_shared<nodes::MotorNode>();
    executor->add_node(motor);

    auto line_sensors = std::make_shared<nodes::LineNode>();
    executor->add_node(line_sensors);

    auto line_loop = std::make_shared<LineLoop>(line_sensors, motor);
    executor->add_node(line_loop);

    auto executor_thread = std::thread([& executor](){executor->spin();});
    while (rclcpp::ok())
    {

    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
