#include "carmen_hardware/robot_hardware.h"
#include <termios.h>
#include <string.h>
#include "carmen_hardware/protocol.h"
#include <hardware_interface/robot_hw.h>
#include <tf2/LinearMath/Quaternion.h>

namespace carmen_hardware
{

  CarmenRobotHW::CarmenRobotHW()
  {
  }

  CarmenRobotHW::~CarmenRobotHW()
  {
  }

  void CarmenRobotHW::setupHardwareInterfaces()
  {
    hardware_interface::JointStateHandle front_right_wheel_joint_state_handle("front_right_wheel_joint",
      &position_[0], &velocity_[0], &effort_[0]);
    joint_state_interface_.registerHandle(front_right_wheel_joint_state_handle);

    hardware_interface::JointStateHandle front_left_wheel_joint_state_handle("front_left_wheel_joint",
      &position_[1], &velocity_[1], &effort_[1]);
    joint_state_interface_.registerHandle(front_left_wheel_joint_state_handle);

    hardware_interface::JointStateHandle rear_right_wheel_joint_state_handle("rear_right_wheel_joint",
      &position_[2], &velocity_[2], &effort_[2]);
    joint_state_interface_.registerHandle(rear_right_wheel_joint_state_handle);

    hardware_interface::JointStateHandle rear_left_wheel_joint_state_handle("rear_left_wheel_joint",
      &position_[3], &velocity_[3], &effort_[3]);
    joint_state_interface_.registerHandle(rear_left_wheel_joint_state_handle);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle front_right_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("front_right_wheel_joint"), &command_[0]);
    joint_velocity_interface_.registerHandle(front_right_wheel_joint_velocity_handler);

    hardware_interface::JointHandle front_left_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("front_left_wheel_joint"), &command_[1]);
    joint_velocity_interface_.registerHandle(front_left_wheel_joint_velocity_handler);

    hardware_interface::JointHandle rear_right_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("rear_right_wheel_joint"), &command_[2]);
    joint_velocity_interface_.registerHandle(rear_right_wheel_joint_velocity_handler);

    hardware_interface::JointHandle rear_left_wheel_joint_velocity_handler(
        joint_state_interface_.getHandle("rear_left_wheel_joint"), &command_[3]);
    joint_velocity_interface_.registerHandle(rear_left_wheel_joint_velocity_handler);

    registerInterface(&joint_velocity_interface_);

    carmen_control::RangeSensorHandle left_sonar_handle("left_sonar", &sonar_[0]);
    range_sensor_interface_.registerHandle(left_sonar_handle);

    carmen_control::RangeSensorHandle central_sonar_handle("central_sonar", &sonar_[1]);
    range_sensor_interface_.registerHandle(central_sonar_handle);

    carmen_control::RangeSensorHandle right_sonar_handle("right_sonar", &sonar_[2]);
    range_sensor_interface_.registerHandle(right_sonar_handle);
    registerInterface(&range_sensor_interface_);

    hardware_interface::ImuSensorHandle imu_handle("imu", "imu_link", imu_orientation_, imu_orientation_covariances_,
      imu_angular_velocity_, imu_angular_velocity_covariances_,
      imu_linear_acceleration_, imu_linear_acceleration_covariances_);
    imu_sensor_interface_.registerHandle(imu_handle);
    registerInterface(&imu_sensor_interface_);
  }

  void CarmenRobotHW::sendHandshake()
  {
    HandshakeCommand command;
    HandshakeResult result;
    try
    {
      orion_major_.invoke(command, &result, orion::Major::Interval::Second, 3);
      ROS_INFO("Handshake success!");
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("Error during hand shake: " << e.what() << "\n");
    }
  }

  bool CarmenRobotHW::init(ros::NodeHandle& root_nh)
  {
    this->setupHardwareInterfaces();

    realtime_magnetic_field_publisher_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::MagneticField>(root_nh,
      "/mag_field", 4));

    std::string default_port = "/dev/ttyACM0";
    std::string port;
    root_nh.param<std::string>("port", port, default_port);
    // TODO(Andriy): Add reading of baud
    this->serial_port.connect(port.c_str(), B230400);
    this->sendHandshake();
  }

  void CarmenRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
  }

  void CarmenRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    SetCommandsCommand command;
    SetCommandsResult result;

    command.right_cmd = static_cast<int16_t>(command_[0] * 1000);
    command.left_cmd = static_cast<int16_t>(command_[1] * 1000);
    try
    {
      orion_major_.invoke(command, &result, 250 * orion::Major::Interval::Millisecond, 1);
      velocity_[0] = result.wheel_vel_right / 1000.0;
      velocity_[1] = result.wheel_vel_left / 1000.0;
      velocity_[2] = velocity_[0]; 
      velocity_[3] = velocity_[1]; 

      position_[0] = result.wheel_pos_right / 1000.0;
      position_[1] = result.wheel_pos_left / 1000.0;
      position_[2] = position_[0];
      position_[3] = position_[1];

      sonar_[0] = result.ultra_sonic_left / 1000.0;
      sonar_[1] = result.ultra_sonic_center / 1000.0;
      sonar_[2] = result.ultra_sonic_right / 1000.0;

      if (realtime_magnetic_field_publisher_->trylock()) {

        realtime_magnetic_field_publisher_->msg_.header.stamp = time;
        realtime_magnetic_field_publisher_->msg_.header.frame_id = "imu_link";

        realtime_magnetic_field_publisher_->msg_.magnetic_field.x = result.imu_angle_beta / 1000000000.0;
        realtime_magnetic_field_publisher_->msg_.magnetic_field.y = result.imu_angle_alpha / 1000000000.0;
        realtime_magnetic_field_publisher_->msg_.magnetic_field.z = -1.0 * result.imu_angle_gamma / 1000000000.0;
        for (int i=0; i < realtime_magnetic_field_publisher_->msg_.magnetic_field_covariance.size(); i++) {
          realtime_magnetic_field_publisher_->msg_.magnetic_field_covariance[i] = mag_sensor_covariances_[i];
        }

        realtime_magnetic_field_publisher_->unlockAndPublish();
      }

      imu_angular_velocity_[0] = result.imu_vel_beta / 1000.0;
      imu_angular_velocity_[1] = result.imu_vel_alpha / 1000.0;
      imu_angular_velocity_[2] = -1.0 * result.imu_vel_gamma / 1000.0;

      imu_linear_acceleration_[0] = result.imu_acc_y / 1000.0;
      imu_linear_acceleration_[1] = result.imu_acc_x / 1000.0;
      imu_linear_acceleration_[2] = -1.0 * result.imu_acc_z / 1000.0;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Error during control loop: " << e.what() << "\n");
    }
  }

}  // carmen_hardware
