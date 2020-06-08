#include <ros/ros.h>
#include "carmen_hardware/robot_hardware.h"
#include <controller_manager/controller_manager.h>

using carmen_hardware::CarmenRobotHW;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carmen_hardware_node");
  ros::NodeHandle node_handle;

  CarmenRobotHW robot;
  robot.init(node_handle);
  controller_manager::ControllerManager controller_manager(&robot, node_handle);

  ros::Time ts = ros::Time::now();

  ros::Rate rate(10);
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