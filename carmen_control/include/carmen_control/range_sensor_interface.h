#ifndef CARMEN_CONTROL_RANGE_SENSOR_INTERFACE_H
#define CARMEN_CONTROL_RANGE_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace carmen_control
{

class RangeSensorHandle
{
public:

  RangeSensorHandle() = default;

  RangeSensorHandle(
        const std::string& name,
        const std::string& frame_id,
        const double* range
    )
    : name_(name),
      frame_id_(frame_id),
      range_(range)
  {}

  std::string getName()    const { return name_; }
  std::string getFrameId() const { return frame_id_; }
  double getRange() const { return *range_; }

private:
  std::string name_;
  std::string frame_id_;
  const double* range_ = NULL;
};

class RangeSensorInterface : public hardware_interface::HardwareResourceManager<RangeSensorHandle> {};

}  // carmen_control

#endif  // CARMEN_CONTROL_RANGE_SENSOR_INTERFACE_H
