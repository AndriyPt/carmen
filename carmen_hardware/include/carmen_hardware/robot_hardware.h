#ifndef CARMEN_HARDWARE_ROBOT_HARDWARE_H
#define CARMEN_HARDWARE_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <orion_protocol/orion_major.hpp>
#include <orion_protocol/orion_transport.hpp>
#include <orion_protocol/orion_serial_port.hpp>
#include <carmen_control/range_sensor_interface.h>

namespace carmen_hardware
{

class CarmenRobotHW : public hardware_interface::RobotHW
{
public:
  CarmenRobotHW();
  virtual ~CarmenRobotHW();
  virtual bool init(ros::NodeHandle& root_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  void sendHandshake();
  void setupHardwareInterfaces();

  hardware_interface::JointStateInterface joint_state_interface_;
  carmen_control::RangeSensorInterface range_sensor_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  static const uint8_t JOINTS_COUNT = 4;

  double command_[JOINTS_COUNT];
  double position_[JOINTS_COUNT];
  double velocity_[JOINTS_COUNT];
  double effort_[JOINTS_COUNT];

  static const uint8_t SONARS_COUNT = 3;
  double sonar_[SONARS_COUNT];

  uint32_t control_loop_timeout;

  orion::SerialPort serial_port_ = orion::SerialPort();
  orion::Transport frame_transport_ = orion::Transport(&this->serial_port_);
  orion::Major orion_major_ = orion::Major(&this->frame_transport_);
};

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_ROBOT_HARDWARE_H
