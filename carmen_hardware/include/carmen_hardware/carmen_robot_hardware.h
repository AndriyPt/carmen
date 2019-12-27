#ifndef CARMEN_HARDWARE_CARMEN_ROBOT_HARDWARE_H
#define CARMEN_HARDWARE_CARMEN_ROBOT_HARDWARE_H

#include <atomic>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "carmen_msgs/FirmwareStateRead.h"

namespace carmen_hardware
{

enum MotorSide {Left = 0, Right = 1, EnumSize = 2};

class CarmenRobotHW : public hardware_interface::RobotHW
{
public:
  CarmenRobotHW();
  virtual ~CarmenRobotHW();
  virtual bool init(ros::NodeHandle& root_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  void jointStateCallback(const carmen_msgs::FirmwareStateRead::ConstPtr& message);

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface joint_velocity_interface;

  ros::Subscriber motor_joint_state;
  ros::Publisher publish_velocity;

  double[MotorSide::EnumSize] command;
  double[MotorSide::EnumSize] position;
  double[MotorSide::EnumSize] velocity;
  double[MotorSide::EnumSize] effort;

  std::atomic<double>[MotorSide::EnumSize] hardware_motor_position;
  std::atomic<double>[MotorSide::EnumSize] hardware_motor_velocity;
  std::atomic<double>[MotorSide::EnumSize] hardware_motor_effort;
};

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_CARMEN_ROBOT_HARDWARE_H
