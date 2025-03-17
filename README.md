# README
## To build, run in terminal:
    source /opt/ros/humble/setup.bash
    clion

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