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
    std::string port = "/dev/ttyACM1";
    root_nh.param<std::string>("port", port, port);
    // TODO(Andriy): Add reading of baud
    this->serial_port.connect(port, B230400);
    this->sendHandshake();
  }

  void CarmenRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
  }

  void CarmenRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    SetCommandsCommand command;
    SetCommandsResult result;
    command.left_cmd = 10000;
    command.right_cmd = 20;
    try
    {
      orion_major_.invoke(command, &result, 250 * orion::Major::Interval::Millisecond, 1);
      if (result.result)
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