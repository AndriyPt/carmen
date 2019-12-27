#include "carmen_hardware/carmen_robot_hardware.h"
#include <std_msgs/Float32.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <vector>

namespace carmen_hardware
{

  CarmenRobotHW::CarmenRobotHW()
  {
  }

  CarmenRobotHW::~CarmenRobotHW()
  {
  }

  void CarmenRobotHW::jointStateCallback(const carmen_msgs::JointState::ConstPtr& message)
  {
    for (unsigned int i = 0; i < MotorSide::EnumSize; i++)
    {
      hardware_motor_position[i] = message->position;
      hardware_motor_velocity[i] = message->velocity;
      hardware_motor_effort[i] = message->effort;
    }
  }

  bool CarmenRobotHW::init(ros::NodeHandle& root_nh)
  {
    publish_velocity = root_nh.advertise<std_msgs::Float32>("hardware_set_motor_velocity", 1000);
    motor_joint_state = root_nh.subscribe("hardware_motor_state", 1000, &CarmenRobotHW::jointStateCallback, this);

    hardware_interface::JointStateHandle wheel_joint_state_handle("wheel_joint", &position, &velocity, &effort);
    joint_state_interface.registerHandle(wheel_joint_state_handle);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle wheel_joint_effort_handler(joint_state_interface.getHandle("wheel_joint"),
      &command);
    joint_effort_interface.registerHandle(wheel_joint_effort_handler);

    registerInterface(&joint_effort_interface);
  }

  void CarmenRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    position = hardware_motor_position;
    velocity = hardware_motor_velocity;
    effort = hardware_motor_effort;
  }

  void CarmenRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    std_msgs::Float32 message;
    message.data = command;
    publish_effort.publish(message);
  }

}  // carmen_hardware
