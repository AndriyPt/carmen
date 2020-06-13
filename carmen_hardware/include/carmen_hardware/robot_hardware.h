#ifndef CARMEN_HARDWARE_ROBOT_HARDWARE_H
#define CARMEN_HARDWARE_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <orion_protocol/orion_master.h>

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

  orion::Master orion_master_ = orion::Master(nullptr);
};

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_ROBOT_HARDWARE_H
