#ifndef CARMEN_CONTROL_RANGE_SENSOR_CONTROLLER_H
#define CARMEN_CONTROL_RANGE_SENSOR_CONTROLLER_H

#include <controller_interface/controller.h>
#include "carmen_control/range_sensor_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Range.h>
#include <realtime_tools/realtime_publisher.h>
#include <stdint.h>
#include <memory>
#include <string>

namespace carmen_control
{

class RangeSensorController: public controller_interface::Controller<RangeSensorInterface>
{
public:
  RangeSensorController() {};

  virtual bool init(RangeSensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  RangeSensorHandle sensor_;
  typedef std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Range> > RtPublisherPtr;
  RtPublisherPtr realtime_pub_;
  ros::Time last_publish_time_;

  double publish_rate_;     // Publish rate
  std::string frame_id_;    // Frame Id of the range
  uint8_t radiation_type_;  // The type of radiation used by the sensor (0 = sound, 1 = IR)
  double field_of_view_;    // The size of the arc that the distance reading is valid for
  double min_range_;        // Minimum range value
  double max_range_;        // Maximum range value
};

}  // carmen_control

#endif  // CARMEN_CONTROL_RANGE_SENSOR_CONTROLLER_H
