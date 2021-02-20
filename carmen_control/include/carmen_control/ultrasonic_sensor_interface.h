#ifndef CARMEN_CONTROL_ULTRASONIC_SENSOR_INTERFACE_H
#define CARMEN_CONTROL_ULTRASONIC_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <stdint.h>

namespace carmen_control
{

class UltrasonicSensorHandle
{
public:
  struct Data
  {
    // Note: User-provided constructor required due to a defect in the standard. See https://stackoverflow.com/a/17436088/1932358
    Data() {}

    std::string name;           // The name of the sensor
    std::string frame_id;       // The reference frame to which this sensor is associated
    uint8_t radiation_type;     // The type of radiation used by the sensor (0 = sound, 1 = IR)
    double field_of_view;       // The size of the arc that the distance reading is valid for
    double min_range;           // Minimum range value
    double max_range;           // Maximum range value [m]
    double* range = {nullptr};  // A pointer to the storage of the range data       
  };

  UltrasonicSensorHandle(const Data& data = {})
    : name_(data.name),
      frame_id_(data.frame_id),
      radiation_type_(data.radiation_type),
      field_of_view_(data.field_of_view),
      min_range_(data.min_range),
      max_range_(data.max_range),
      range_(data.range)
  {}

  UltrasonicSensorHandle(
        const std::string& name,
        const std::string& frame_id,
        const uint8_t radiation_type,
        const double field_of_view,
        const double min_range,
        const double max_range,
        const double* range
    )
    : name_(name),
      frame_id_(frame_id),
      radiation_type_(radiation_type),
      field_of_view_(field_of_view),
      min_range_(min_range),
      max_range_(max_range),
      range_(range)
  {}

  std::string getName()      const {return name_;}
  std::string getFrameId()   const {return frame_id_;}
  uint8_t getRadiationType() const {return radiation_type_;}
  double getFieldOfView()    const {return field_of_view_;}
  double getMinRange()       const {return min_range_;}
  double getMaxRange()       const {return max_range_;}
  const double* getRange()   const {return range_;}

private:
  std::string name_;
  std::string frame_id_;
  uint8_t radiation_type_;
  double field_of_view_;
  double min_range_;
  double max_range_;

  const double* range_;
};

class UltrasonicSensorInterface : public HardwareResourceManager<UltrasonicSensorHandle> {};

}  // carmen_control

#endif  // CARMEN_CONTROL_ULTRASONIC_SENSOR_INTERFACE_H
