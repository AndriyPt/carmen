#include <ros/ros.h>
#include <stdint.h>
#include "carmen_hardware/robot_hardware.h"
#include <controller_manager/controller_manager.h>

using carmen_hardware::CarmenRobotHW;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carmen_hardware_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  CarmenRobotHW robot;
  robot.init(private_node_handle);
  controller_manager::ControllerManager controller_manager(&robot, node_handle);

  ros::Time ts = ros::Time::now();

  int32_t rate_value = 10;
  private_node_handle.param<int32_t>("rate", rate_value, rate_value);

  ros::Rate rate(rate_value);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read(ts, d);
     controller_manager.update(ts, d);
     robot.write(ts, d);
     rate.sleep();
  }

  return 0;
}