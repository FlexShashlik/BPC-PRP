#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "SineWave.h"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // // Create multiple nodes
    // auto node1 = std::make_shared<rclcpp::Node>("node1");
    // auto node2 = std::make_shared<rclcpp::Node>("node2");
    //
    // // Create instances of RosExampleClass using the existing nodes
    // auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", .1);
    // auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);
    //
    // // Add nodes to the executor
    // executor->add_node(node1);
    // executor->add_node(node2);
    //
    // auto nodeSineWave = std::make_shared<rclcpp::Node>("nodeSineWave");
    // auto sineWave = std::make_shared<SineWave>(nodeSineWave, "sineWave", "topic1");
    // executor->add_node(nodeSineWave);

    // Run the executor (handles callbacks for both nodes)
    //executor->spin();

    auto io = std::make_shared<nodes::IoNode>();
    executor->add_node(io);

    auto motor = std::make_shared<nodes::MotorNode>();
    executor->add_node(motor);

    auto executor_thread = std::thread([& executor](){executor->spin();});
    while (rclcpp::ok())
    {
        // Create a message of type UInt8MultiArray
        auto rgb_message = std_msgs::msg::UInt8MultiArray();

        // Set up the layout (dimensions)
        rgb_message.layout.dim.resize(1);
        rgb_message.layout.dim[0].label = "LEDs";
        rgb_message.layout.dim[0].size = 4;     // 4 LEDs
        rgb_message.layout.dim[0].stride = 12;   // 12 because 3 values per LED
        rgb_message.layout.data_offset = 0;

        auto motor_message = std_msgs::msg::UInt8MultiArray();

        // Set up the layout (dimensions)
        motor_message.layout.dim.resize(1);
        motor_message.layout.dim[0].label = "SPEEDs";
        motor_message.layout.dim[0].size = 2;     // 4 LEDs
        motor_message.layout.dim[0].stride = 2;   // 12 because 3 values per LED
        motor_message.layout.data_offset = 0;

        float intensity_phase_;

        switch (io->get_button_pressed()) {
            case 0:
                {
                    // Set the RGB data for the LEDs (4 LEDs, each with 3 values: R, G, B)
                    rgb_message.data = {255, 0, 0,    // LED 1: Red
                                    0, 255, 0,    // LED 2: Green
                                    0, 0, 255,    // LED 3: Blue
                                    255, 255, 255}; // LED 4: White
                    io->rgb_led_publish(rgb_message);

                    // --- MOTOR TEST

                    motor_message.data = {255, 255}; // LED 4: White
                    motor->motor_speeds_publish(motor_message);

                }
                break;
            case 1:
                {
                // Set the RGB data for the LEDs (4 LEDs, each with 3 values: R, G, B)
                    rgb_message.data = {255, 0, 0,    // LED 1: Red
                                        0, 255, 0,    // LED 2: Green
                                        0, 0, 255,    // LED 3: Blue
                                        255, 255, 255}; // LED 4: White
                io->rgb_led_publish(rgb_message);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                // Set the RGB data for the LEDs (4 LEDs, each with 3 values: R, G, B)
                rgb_message.data = {255, 0, 0,
                                0, 0, 255,
                                255, 255, 255,
                                0, 255, 0};
                io->rgb_led_publish(rgb_message);
                }
                break;
            case 2: {
                intensity_phase_ += 0.1;
                if (intensity_phase_ > 2 * M_PI) {
                    intensity_phase_ -= 2 * M_PI;
                }

                float rgb[3] = {};

                // Create sine wave intensity values for each LED
                for (int i = 0; i < 4; ++i) {
                    float intensity = (std::sin(intensity_phase_ + (i * M_PI / 3.0)) + 1.0) / 2.0; // Phase shift each LED
                    rgb[i] = static_cast<uint8_t>(255 * intensity); // Scale to 255
                }

                rgb_message.data = {rgb[0], rgb[1], rgb[2],    // LED 1: Red
                                rgb[0], rgb[1], rgb[2],  // LED 3: Blue
                                rgb[0], rgb[1], rgb[2], // LED 4: White
                                rgb[0], rgb[1], rgb[2]};    // LED 2: Green
                io->rgb_led_publish(rgb_message);
                }
                break;
            default:
                break;
        };

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
