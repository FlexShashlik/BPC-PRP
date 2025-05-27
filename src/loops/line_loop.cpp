//
// Created by student on 24.3.25.
//
#include "loops/line_loop.hpp"

#include <chrono>
#include <thread>

#include "helper.hpp"
#include "kinematics.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/motor_node.hpp"

LineLoop::LineLoop (std::shared_ptr<nodes::CameraNode> camera, std::shared_ptr<nodes::ImuNode> imu, std::shared_ptr<nodes::MotorNode> motor) : Node(
        "lineLoopNode"), pid_(3, 0, 0), last_time_(this->now()), kinematics_(0.033, 0.16, 360) {
    this->get_logger().set_level(rclcpp::Logger::Level::Warn);

    lidar_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                Topic::lidar_avg, 1,
                std::bind(&LineLoop::on_lidar_msg, this, std::placeholders::_1));

    motor_ = motor;

    front_limit_ = MIN_FRONT_DISTANCE;
    imu_ = imu;
    camera_ = camera;

    isCalibrated_ = false;

    nextMove_ = ArucoType::None;

    integral_ = 0;
    previous_error_ = 0;
    kp_ = 1.5f, ki_ = 0, kd_ = 0;

    base_linear_velocity_ = 0.04f;
    lidar_results_ = {};

    last_time_ = this->now();
}

void LineLoop::Restart() {
    pid_.reset();
    last_time_ = this->now();
}

LineLoopState LineLoop::getState() const {
    return state_;
}

void LineLoop::TurnLeft(TurnStage turn_stage)
{
    yaw_ref_ = rad2deg(deg2rad(yaw_start_) + (M_PI / 2));
    state_ = LineLoopState::TURNING;
    turn_stage_ = turn_stage;
    if (turn_stage == TurnStage::FORWARD_TO_EDGE)
        RCLCPP_WARN(this->get_logger(), "FORWARD TO EDGE");
    RCLCPP_WARN(this->get_logger(), "TURNING LEFT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
}

void LineLoop::TurnRight(TurnStage turn_stage)
{
    yaw_ref_ = rad2deg(deg2rad(yaw_start_) - (M_PI / 2));
    state_ = LineLoopState::TURNING;
    turn_stage_ = turn_stage;
    if (turn_stage == TurnStage::FORWARD_TO_EDGE)
        RCLCPP_WARN(this->get_logger(), "FORWARD TO EDGE");
    RCLCPP_WARN(this->get_logger(), "TURNING RIGHT from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
}

void LineLoop::Turn180(TurnStage turn_stage)
{
    yaw_ref_ = yaw_start_ + 180;
    state_ = LineLoopState::TURNING;
    turn_stage_ = turn_stage;
    if (turn_stage == TurnStage::FORWARD_TO_EDGE)
        RCLCPP_WARN(this->get_logger(), "FORWARD TO EDGE");
    RCLCPP_WARN(this->get_logger(), "TURNING 180 from: %.2f°,  %.2f°", yaw_start_, yaw_ref_);
}

float LineLoop::calculate_pid_angular_velocity(float left_dist, float right_dist)
{
    // Filter out small differences in raw measurements
    float left = left_dist;
    float right = right_dist;
    const float distance_deadband = 0.02f; // 2cm deadband for raw distances

    if (std::abs(left - right) < distance_deadband)
    {
        // If difference is smaller than deadband, consider them equal
        left = (left + right) / 2.0f;
        right = left;
    }

    float corridor_offset = left - right;

    float error = corridor_offset;
    integral_ += error * 0.02f;
    float derivative = (error - previous_error_) / 0.02f;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    previous_error_ = error;
    return output;
}

float LineLoop::calculate_left_wall_pid_angular_velocity(float left_dist)
{
    return calculate_pid_angular_velocity(left_dist, WALL_DISTANCE);
}

float LineLoop::calculate_right_wall_pid_angular_velocity(float right_dist)
{
    return calculate_pid_angular_velocity(WALL_DISTANCE, right_dist);
}

void LineLoop::maze_loop(float dtMS) {
    if (imu_->getMode() == nodes::ImuNodeMode::CALIBRATE)
    {
        state_ = LineLoopState::CALIBRATION;
        return;
    }
    if (!isCalibrated_ && imu_->getMode() != nodes::ImuNodeMode::CALIBRATE)
    {
        RCLCPP_WARN(this->get_logger(), "IMU is calibrated");
        isCalibrated_ = true;
        state_ = LineLoopState::CORRIDOR_FOLLOWING;
        RCLCPP_INFO(this->get_logger(), "State CORRIDOR_FOLLOWING");
    }

    switch (state_)
    {
        case LineLoopState::CALIBRATION:
            // Wait until enough samples are collected
            // Once done, switch to CORRIDOR_FOLLOWING
        break;

        case LineLoopState::CORRIDOR_FOLLOWING:
        // Keep centered using P/PID based on side distances
        // If front is blocked and one side is open → switch to TURNING
        {
            if (lidar_results_.front == -1 || lidar_results_.right == -1 || lidar_results_.left == -1 || lidar_results_.front_right == -1 || lidar_results_.front_left == -1)
            {
                RCLCPP_INFO(this->get_logger(), "Empty, continue..");

                if (lidar_results_.left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty left, too close probably, substitute..");
                    lidar_results_.left = 0;
                }

                if (lidar_results_.right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty right, too close probably, substitute..");
                    lidar_results_.right = 0;
                }

                if (lidar_results_.front_right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_right, too close probably, substitute..");
                    lidar_results_.front_right = 0;
                }

                if (lidar_results_.front_left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_left, too close probably, substitute..");
                    lidar_results_.front_left = 0;
                }

                if (lidar_results_.front == -1)
                    return;
            }

            if (lidar_results_.front > FRONT_WALL_DISTANCE)
            {
                if (lidar_results_.isRightClosed && lidar_results_.isLeftClosed)
                {
                    float angular_velocity = calculate_pid_angular_velocity(lidar_results_.left, lidar_results_.right);
                    float linear_velocity = base_linear_velocity_;

                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

                    motor_->go(convert_speed_to_command(wheel_speeds.l), convert_speed_to_command(wheel_speeds.r));
                }
                else if (lidar_results_.isRightOpen && lidar_results_.isLeftClosed)
                {
                    if (lidar_results_.front < MIN_FRONT_DISTANCE || !doTurn(getNextMove()))
                    {
                        RCLCPP_WARN(this->get_logger(), "PID using left wall");
                        float angular_velocity = calculate_left_wall_pid_angular_velocity(lidar_results_.wide_left);
                        float linear_velocity = base_linear_velocity_;

                        algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

                        motor_->go(convert_speed_to_command(wheel_speeds.l), convert_speed_to_command(wheel_speeds.r));
                    }
                }
                else if (lidar_results_.isRightClosed && lidar_results_.isLeftOpen)
                {
                    if (lidar_results_.front < MIN_FRONT_DISTANCE || !doTurn(getNextMove()))
                    {
                        RCLCPP_WARN(this->get_logger(), "PID using right wall");
                        float angular_velocity = calculate_right_wall_pid_angular_velocity(lidar_results_.wide_right);
                        float linear_velocity = base_linear_velocity_;

                        algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                        algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

                        motor_->go(convert_speed_to_command(wheel_speeds.l), convert_speed_to_command(wheel_speeds.r));
                    }
                }
                else
                {
                    // X-section
                    // If less than min distance to be a real X-section (can be just going to T-section)
                    if (lidar_results_.front > MIN_FRONT_DISTANCE)
                    {
                        motor_->go(135, 135);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "X-section error");
                        motor_->stop();
                        /*
                        motor_->stop();
                        Turn180();*/
                    }
                }
            }
            else // Closed turns (no way forward)
            {
                if (nextMove_ == ArucoType::Straight || nextMove_ == ArucoType::TreasureStraight)
                    nextMove_ = ArucoType::None;

                yaw_start_ = imu_->getIntegratedResults();
                if (lidar_results_.isLeftClosed && lidar_results_.isRightClosed)
                {
                    // Dead end
                    Turn180();
                }
                else if (lidar_results_.isLeftOpen && lidar_results_.isRightClosed)
                {
                    TurnLeft();
                }
                else if (lidar_results_.isRightOpen && lidar_results_.isLeftClosed)
                {
                    TurnRight();
                }
                else
                {
                    // T-turn
                    if (!doTurn(getNextMove()))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Can't go straight in a T-turn!");
                        motor_->go(127, 127);
                        motor_->stop();
                    }
                }
            }
        }
        break;

        case LineLoopState::AFTER_TURNING:
        {
            if (lidar_results_.front == -1 || lidar_results_.right == -1 || lidar_results_.left == -1 || lidar_results_.front_right == -1 || lidar_results_.front_left == -1)
            {
                RCLCPP_INFO(this->get_logger(), "Empty, continue..");

                if (lidar_results_.left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty left, too close probably, substitute..");
                    lidar_results_.left = 0;
                }

                if (lidar_results_.right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty right, too close probably, substitute..");
                    lidar_results_.right = 0;
                }

                if (lidar_results_.front_right == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_right, too close probably, substitute..");
                    lidar_results_.front_right = 0;
                }

                if (lidar_results_.front_left == -1) {
                    RCLCPP_INFO(this->get_logger(), "Empty front_left, too close probably, substitute..");
                    lidar_results_.front_left = 0;
                }

                if (lidar_results_.front == -1)
                    return;
            }

            if (lidar_results_.front > FRONT_WALL_DISTANCE)
            {
                if (lidar_results_.isRightClosed && lidar_results_.isLeftClosed)
                {
                    state_ = LineLoopState::CORRIDOR_FOLLOWING;
                    RCLCPP_WARN(this->get_logger(), "State CORRIDOR FOLLOWING");
                }
                else if (lidar_results_.isRightOpen && lidar_results_.isLeftClosed)
                {
                    RCLCPP_WARN(this->get_logger(), "PID using left wall");
                    float angular_velocity = calculate_left_wall_pid_angular_velocity(lidar_results_.wide_left);
                    float linear_velocity = base_linear_velocity_;

                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

                    motor_->go(convert_speed_to_command(wheel_speeds.l), convert_speed_to_command(wheel_speeds.r));
                }
                else if (lidar_results_.isRightClosed && lidar_results_.isLeftOpen)
                {
                    RCLCPP_WARN(this->get_logger(), "PID using right wall");
                    float angular_velocity = calculate_right_wall_pid_angular_velocity(lidar_results_.wide_right);
                    float linear_velocity = base_linear_velocity_;

                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

                    motor_->go(convert_speed_to_command(wheel_speeds.l), convert_speed_to_command(wheel_speeds.r));
                }
                else
                {
                    motor_->go(135, 135);
                }
            }
            else
            {
                if (nextMove_ == ArucoType::Straight || nextMove_ == ArucoType::TreasureStraight)
                    nextMove_ = ArucoType::None;

                yaw_start_ = imu_->getIntegratedResults();
                if (lidar_results_.isLeftClosed && lidar_results_.isRightClosed)
                {
                    // Dead end
                    Turn180();
                }
                else if (lidar_results_.isLeftOpen && lidar_results_.isRightClosed)
                {
                    TurnLeft();
                }
                else if (lidar_results_.isRightOpen && lidar_results_.isLeftClosed)
                {
                    TurnRight();
                }
                else
                {
                    // T-turn
                    if (!doTurn(getNextMove()))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Can't go straight in a T-turn!");
                        motor_->go(127, 127);
                        motor_->stop();
                    }
                }
            }
        }
        break;

        case LineLoopState::TURNING:
            // Use IMU to track rotation
            // Rotate until yaw changes by ±90°
            // Then return to CORRIDOR_FOLLOWING
        {
            switch (turn_stage_)
            {
                case TurnStage::FORWARD_TO_EDGE:
                {
                    if (lidar_results_.front > initial_front_ - DELTA_TO_TURN &&
                        lidar_results_.front > FRONT_WALL_DISTANCE)
                    {
                        RCLCPP_WARN(this->get_logger(), "%f > %f", lidar_results_.front, initial_front_ - DELTA_TO_TURN);
                        motor_->go(138, 138);  // Keep going forward
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "%f <= %f", lidar_results_.front, initial_front_ - DELTA_TO_TURN);
                        motor_->go(127, 127); // Stop
                        turn_stage_ = TurnStage::ROTATE;
                    }
                }
                break;

                case TurnStage::ROTATE:
                {
                    float current_yaw = imu_->getIntegratedResults();
                    float yaw_error = yaw_ref_ - current_yaw;

                    if (std::abs(yaw_error) < MAX_YAW_ERROR)
                    {
                        state_ = LineLoopState::AFTER_TURNING;
                        turn_stage_ = TurnStage::DONE;
                        RCLCPP_WARN(this->get_logger(), "Finished TURNING: %.2f°", current_yaw);
                        RCLCPP_WARN(this->get_logger(), "State AFTER TURNING yaw_start: %.2f°, yaw_ref: %.2f°", yaw_start_, yaw_ref_);

                        if (nextMove_ == Right || nextMove_ == Left || nextMove_ == TreasureRight || nextMove_ == TreasureLeft)
                            nextMove_ = None;
                    }
                    else
                    {
                        uint8_t l = 127 - (MAX_TURNING_MOTOR_SPEED * sgn(yaw_error));
                        uint8_t r = 127 + (MAX_TURNING_MOTOR_SPEED * sgn(yaw_error));

                        motor_->go(l, r);
                        RCLCPP_INFO(this->get_logger(), "PID :: outL:%u outR:%u", l, r);
                    }
                }
                break;

                case TurnStage::DONE:
                default:
                break;
            }
        }
        break;
    }
}

ArucoType LineLoop::getNextMove()
{
    if (nextMove_ != ArucoType::None)
        return nextMove_;

    nextMove_ = camera_->GetNextMove();
    RCLCPP_WARN(this->get_logger(), "Next move: %i", (int)nextMove_);
    return nextMove_;
}

bool LineLoop::doTurn(const ArucoType nextMove)
{
    yaw_start_ = imu_->getIntegratedResults();
    initial_front_ = lidar_results_.front;

    switch (nextMove)
    {
        case ArucoType::TreasureLeft:
        case ArucoType::Left:
            TurnLeft(TurnStage::FORWARD_TO_EDGE);
            return true;

        case ArucoType::TreasureRight:
        case ArucoType::Right:
            TurnRight(TurnStage::FORWARD_TO_EDGE);
            return true;

        case ArucoType::Straight:
        case ArucoType::TreasureStraight:
        default:
            return false;
    }
}

void LineLoop::on_lidar_msg(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 5)
        return;

    lidar_results_.front = msg->data[0];
    lidar_results_.right = msg->data[1];
    lidar_results_.left = msg->data[2];
    lidar_results_.wide_right = msg->data[3];
    lidar_results_.wide_left = msg->data[4];
    lidar_results_.isRightOpen = static_cast<bool>(msg->data[5]);
    lidar_results_.isLeftOpen = static_cast<bool>(msg->data[6]);
    lidar_results_.isRightClosed = static_cast<bool>(msg->data[7]);
    lidar_results_.isLeftClosed = static_cast<bool>(msg->data[8]);
    lidar_results_.isFrontWallFarEnoughToUsePIDForCorridorFollowing = static_cast<bool>(msg->data[9]);

    auto current_time = this->now();
    auto delta = current_time - last_time_;
    float delta_ms = static_cast<float>(delta.nanoseconds()) / 1e6;
    last_time_ = current_time;
    maze_loop(delta_ms);
}
