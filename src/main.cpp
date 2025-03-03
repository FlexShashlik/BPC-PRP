#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "SineWave.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("node1");
    auto node2 = std::make_shared<rclcpp::Node>("node2");

    // Create instances of RosExampleClass using the existing nodes
    auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", .1);
    auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);

    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);

    auto nodeSineWave = std::make_shared<rclcpp::Node>("nodeSineWave");
    auto sineWave = std::make_shared<SineWave>(nodeSineWave, "sineWave", "topic1");
    executor->add_node(nodeSineWave);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
