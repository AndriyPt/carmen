#include "carmen_control/range_sensor_controller.h"
#include <limits>

namespace carmen_control
{

  bool RangeSensorController::init(RangeSensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {

    std::string sensor_name;
    if (!controller_nh.getParam("sensor", sensor_name))
    {
      ROS_ERROR("No sensor name given (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
    }
    sensor_ = hw->getHandle(sensor_name);

    std::string frame_id;
    if (!controller_nh.getParam("frame_id", frame_id))
    {
      frame_id = sensor_name;
    }
    frame_id_ = frame_id;

    if (!controller_nh.getParam("publish_rate", publish_rate_)) {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    int radiation_type_param = 0;
    if (!controller_nh.getParam("radiation_type", radiation_type_param)) {
      ROS_ERROR("Parameter 'radiation_type' not set");
      return false;
    }

    if (radiation_type_param > 1) {
      ROS_ERROR("Parameter 'radiation_type' should be either 0 for ultra sound or 1 for infrared types of sensor");
      return false;
    }
    radiation_type_ = static_cast<uint8_t>(radiation_type_param);

    if (!controller_nh.getParam("field_of_view", field_of_view_)) {
      ROS_ERROR("Parameter 'field_of_view' not set");
      return false;
    }

    min_range_ = 0.0;
    controller_nh.getParam("min_range", min_range_);

    max_range_ = std::numeric_limits<double>::max();
    controller_nh.getParam("max_range", max_range_);

    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Range>(root_nh, sensor_name, 4));
    return true;
  }

  void RangeSensorController::starting(const ros::Time& time)
  {
    last_publish_time_ = time;
  }

  void RangeSensorController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {

      if (realtime_pub_->trylock()) {

        last_publish_time_ += ros::Duration(1.0 / publish_rate_);

        realtime_pub_->msg_.header.stamp = time;
        realtime_pub_->msg_.header.frame_id = frame_id_;

        realtime_pub_->msg_.radiation_type = radiation_type_;
        realtime_pub_->msg_.field_of_view = field_of_view_;
        realtime_pub_->msg_.min_range = min_range_;
        realtime_pub_->msg_.max_range = max_range_;
        realtime_pub_->msg_.range = sensor_.getRange();
        
        realtime_pub_->unlockAndPublish();
      }
    }
  }

  void RangeSensorController::stopping(const ros::Time& /*time*/)
  {}

}  // carmen_control

PLUGINLIB_EXPORT_CLASS(carmen_control::RangeSensorController, controller_interface::ControllerBase)
