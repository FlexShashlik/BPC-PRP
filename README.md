# README
# 🧠 Robot Navigation System (ROS 2 Based)

This project implements a modular robotic system built on the **ROS 2 (Robot Operating System 2)** framework, providing a scalable and distributed architecture for autonomous navigation tasks. Using the **publisher–subscriber communication model**, ROS 2 enables real-time data exchange between loosely coupled components (nodes), each responsible for a specific functionality.

## 🧩 System Architecture Overview

The system is composed of several dedicated ROS 2 nodes:

- **Lidar Node** – Collects and preprocesses data from a 2D LiDAR sensor for environmental awareness.
- **Camera Node** – Handles camera communication and image acquisition, including Aruco marker detection using OpenCV.
- **IO Node** – Manages digital I/O such as button states and LED control.
- **IMU Node** – Processes data from the inertial measurement unit (IMU) to estimate orientation and movement.
- **Line Loop Node** – Acts as the main control loop, coordinating sensor inputs and controlling robot behavior.

## 🔧 Algorithms Module

Additional algorithms are located in the `algorithms/` directory:

- `aruco_detector.hpp` – Implements Aruco tag detection logic.
- `lidar_filtr.hpp` – Filters and processes LiDAR data.
- `pid.hpp` – General-purpose PID controller for motion control and stability.
- `planar_imu_integrator.hpp` – Integrates IMU data for pose estimation.

## ⚙️ Utilities

A utility header `helper.hpp` contains frequently used constants and helper functions:

- Physical robot parameters (e.g., wheel radius, encoder resolution)
- Sensor thresholds and motor limits
- Aruco marker IDs and ROS 2 topic definitions
- Math helpers: `mean`, `deg2rad`, `rad2deg`, `sgn`

This modular architecture ensures:

- **Scalability** – Easily extendable with new components
- **Reusability** – Hardware-agnostic algorithms
- **Maintainability** – Decoupled modules allow targeted debugging
- **Parallel Development** – Team members can work on separate features
- **Robustness** – Isolated nodes reduce risk of system-wide failure

## To run from terminal
```bash
source setup.bash
ros2 run prp_project prp_project
```

## To build in clion, run in terminal:
```bash
source /opt/ros/humble/setup.bash
clion
```

## To connect to robot, ensure that the DOMAIN_ID is correct:
    > student@sd151-student-05:~$ env | grep ROS
    ROS_DOMAIN_ID=8
    > student@sd151-student-05:~$ ssh robot@prp-BLUE

## Encoder gives us number of ticks
        3 periods per motor rotation
        1:48 gear ratio
        2 channels
        2 edges
    => 3 * 48 * 2 * 2 = 576 pulses per rotation


## Calibrate line sensors on black line using
        ros2 topic echo /bpc_prp_robot/line_sensors
        and write to helper.hpp to LINE_SENSOR_MAX_CALIBRATED_BLACK