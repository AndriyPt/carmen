#ifndef CARMEN_HARDWARE_ROBOT_HARDWARE_H
#define CARMEN_HARDWARE_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <orion_protocol/orion_major.h>
#include <orion_protocol/orion_frame_transport.h>
#include <orion_protocol/orion_cobs_framer.h>
#include <orion_protocol/orion_serial_port.h>
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
  hardware_interface::ImuSensorInterface imu_sensor_interface_;

  static const uint8_t JOINTS_COUNT = 4;

  double command_[JOINTS_COUNT];
  double position_[JOINTS_COUNT];
  double velocity_[JOINTS_COUNT];
  double effort_[JOINTS_COUNT];

  static const uint8_t MATRIX_3_BY_3 = 3 * 3;

  // TODO: Read from parameters
//  double imu_orientation_covariances_[MATRIX_3_BY_3] = {0.0003, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0, 0.0003};
  // TODO: Turned off orientation as derived data. Could get estimation from imu_complementary_filter
  double imu_orientation_covariances_[MATRIX_3_BY_3] = {-1.0, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0, 0.0003};
	double imu_angular_velocity_covariances_[MATRIX_3_BY_3] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
	double imu_linear_acceleration_covariances_[MATRIX_3_BY_3] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

  static const uint8_t WORLD_DIMENTION = 3;
  static const uint8_t QUATERNION_DIMENTION = 4;

  double imu_orientation_[QUATERNION_DIMENTION];
  double imu_angular_velocity_[WORLD_DIMENTION];
  double imu_linear_acceleration_[WORLD_DIMENTION];

  static const uint8_t SONARS_COUNT = 3;
  double sonar_[SONARS_COUNT];

  orion::SerialPort serial_port = orion::SerialPort();
  orion::COBSFramer cobs_framer_ = orion::COBSFramer();
  orion::FrameTransport frame_transport_ = orion::FrameTransport(&this->serial_port, &this->cobs_framer_);
  orion::Major orion_major_ = orion::Major(&this->frame_transport_);
};

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_ROBOT_HARDWARE_H
