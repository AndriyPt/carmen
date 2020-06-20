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
    ReadSettingsCommand command;
    ReadSettingsResult result;
    command.data.left_front_p = 10000;
    command.data.left_front_i = 0;
    command.data.left_front_d = 0;

    node_handle.param<int32_t>("left_front_p", command.data.left_front_p, command.data.left_front_p);
    node_handle.param<int32_t>("left_front_i", command.data.left_front_i, command.data.left_front_i);
    node_handle.param<int32_t>("left_front_d", command.data.left_front_d, command.data.left_front_d);

    orion_major_.invoke(command, &result, orion::Major::Interval::Second, 3);

    node_handle.setParam("left_front_p", result.data.left_front_p);
    node_handle.setParam("left_front_i", result.data.left_front_i);
    node_handle.setParam("left_front_d", result.data.left_front_d);
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
  }

}  // carmen_hardware