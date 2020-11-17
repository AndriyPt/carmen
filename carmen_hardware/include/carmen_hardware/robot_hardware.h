#ifndef CARMEN_HARDWARE_ROBOT_HARDWARE_H
#define CARMEN_HARDWARE_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <orion_protocol/orion_major.h>
#include <orion_protocol/orion_frame_transport.h>
#include <orion_protocol/orion_cobs_framer.h>
#include <orion_protocol/orion_serial_port.h>

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
  void initParameters(const ros::NodeHandle& node_handle);

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  static const uint8_t JOINTS_COUNT = 4;

  double command_[JOINTS_COUNT];
  double position_[JOINTS_COUNT];
  double velocity_[JOINTS_COUNT];
  double effort_[JOINTS_COUNT];

  orion::SerialPort serial_port = orion::SerialPort();
  orion::COBSFramer cobs_framer_ = orion::COBSFramer();
  orion::FrameTransport frame_transport_ = orion::FrameTransport(&this->serial_port, &this->cobs_framer_);
  orion::Major orion_major_ = orion::Major(&this->frame_transport_);
};

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_ROBOT_HARDWARE_H
