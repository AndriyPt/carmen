#include "carmen_hardware/robot_hardware.h"
#include <termios.h>
#include <string.h>
#include "carmen_hardware/protocol.h"
#include <hardware_interface/robot_hw.h>


namespace carmen_hardware
{

  CarmenRobotHW::CarmenRobotHW()
  {
  }

  CarmenRobotHW::~CarmenRobotHW()
  {
  }

  void CarmenRobotHW::setupHardwareInterfaces()
  {
    hardware_interface::JointStateHandle front_right_wheel_joint_state_handle("front_right_wheel_joint",
      &position_[0], &velocity_[0], &effort_[0]);
    joint_state_interface_.registerHandle(front_right_wheel_joint_state_handle);

    hardware_interface::JointStateHandle front_left_wheel_joint_state_handle("front_left_wheel_joint",
      &position_[1], &velocity_[1], &effort_[1]);
    joint_state_interface_.registerHandle(front_left_wheel_joint_state_handle);

    hardware_interface::JointStateHandle rear_right_wheel_joint_state_handle("rear_right_wheel_joint",
      &position_[2], &velocity_[2], &effort_[2]);
    joint_state_interface_.registerHandle(rear_right_wheel_joint_state_handle);

    hardware_interface::JointStateHandle rear_left_wheel_joint_state_handle("rear_left_wheel_joint",
      &position_[3], &velocity_[3], &effort_[3]);
    joint_state_interface_.registerHandle(rear_left_wheel_joint_state_handle);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle front_right_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("front_right_wheel_joint"), &command_[0]);
    joint_velocity_interface_.registerHandle(front_right_wheel_joint_velocity_handler);

    hardware_interface::JointHandle front_left_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("front_left_wheel_joint"), &command_[1]);
    joint_velocity_interface_.registerHandle(front_left_wheel_joint_velocity_handler);

    hardware_interface::JointHandle rear_right_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("rear_right_wheel_joint"), &command_[2]);
    joint_velocity_interface_.registerHandle(rear_right_wheel_joint_velocity_handler);

    hardware_interface::JointHandle rear_left_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("rear_left_wheel_joint"), &command_[3]);
    joint_velocity_interface_.registerHandle(rear_left_wheel_joint_velocity_handler);

    registerInterface(&joint_velocity_interface_);
  }

  void CarmenRobotHW::sendHandshake()
  {
    HandshakeCommand command;
    HandshakeResult result;
    try
    {
      orion_major_.invoke(command, &result, orion::Major::Interval::Second, 3);
      ROS_INFO("Handshake success!");
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("Error during hand shake: " << e.what() << "\n");
    }
  }

  bool CarmenRobotHW::init(ros::NodeHandle& root_nh)
  {
    this->setupHardwareInterfaces();

    std::string port = "/dev/ttyACM1";
    root_nh.param<std::string>("port", port, port);
    // TODO(Andriy): Add reading of baud
    this->serial_port.connect(port.c_str(), B230400);
    this->sendHandshake();
  }

  void CarmenRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
  }

  void CarmenRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    SetCommandsCommand command;
    SetCommandsResult result;

    //TODO (Andriy): Implement
    command.left_cmd = 10000;
    command.right_cmd = 20;
    try
    {
      orion_major_.invoke(command, &result, 250 * orion::Major::Interval::Millisecond, 1);
      if (0 == result.header.error_code)
      {
        ROS_INFO_THROTTLE(1, "Control loop running as expected");
      }
      else
      {
        ROS_INFO_THROTTLE(1, "Jitter error on commands");
      }
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Error during control loop: " << e.what() << "\n");
    }
  }

}  // carmen_hardware