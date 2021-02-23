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
        const double* range
    )
    : name_(name),
      range_(range)
  {}

  std::string getName()    const { return name_; }
  double getRange() const { return *range_; }

private:
  std::string name_;
  const double* range_ = NULL;
};

class RangeSensorInterface : public hardware_interface::HardwareResourceManager<RangeSensorHandle> {};

}  // carmen_control

#endif  // CARMEN_CONTROL_RANGE_SENSOR_INTERFACE_H
