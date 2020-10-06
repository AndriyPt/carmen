#include "carmen_hardware/robot_hardware.h"
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
    orion_major_.invoke(command, &result, orion::Major::Interval::Second, 3);
  }

  void CarmenRobotHW::initParameters(const ros::NodeHandle& node_handle)
  {
    // ReadSettingsCommand command;
    // ReadSettingsResult result;
    // command.data.left_front_p = 10000;
    // command.data.left_front_i = 0;
    // command.data.left_front_d = 0;

    // node_handle.param<int32_t>("left_front_p", command.data.left_front_p, command.data.left_front_p);
    // node_handle.param<int32_t>("left_front_i", command.data.left_front_i, command.data.left_front_i);
    // node_handle.param<int32_t>("left_front_d", command.data.left_front_d, command.data.left_front_d);

    // orion_major_.invoke(command, &result, orion::Major::Interval::Second, 3);

    // node_handle.setParam("left_front_p", result.data.left_front_p);
    // node_handle.setParam("left_front_i", result.data.left_front_i);
    // node_handle.setParam("left_front_d", result.data.left_front_d);

    SetPIDCommand command;
    SetPIDResult result;
    command.left_p = 10000;
    command.left_i = 0;
    command.left_d = 0;

    command.right_p = 10000;
    command.right_i = 0;
    command.right_d = 0;

    orion_major_.invoke(command, &result, 500 * orion::Major::Interval::Millisecond, 2);

    if (result.result)
    {
      ROS_INFO("Set PID success!");
    }
    else
    {
      ROS_INFO("Set PID failed!");
    }
  }

  bool CarmenRobotHW::init(ros::NodeHandle& root_nh)
  {
    this->sendHandshake();
    this->initParameters(root_nh);
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

    orion_major_.invoke(command, &result, 100 * orion::Major::Interval::Millisecond, 1);
    if (!result.result)
    {
      ROS_INFO_THROTTLE(1, "Jitter error on commands");
    }
  }

}  // carmen_hardware