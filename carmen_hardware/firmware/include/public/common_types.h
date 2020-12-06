#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

namespace carmen_hardware 
{

class ImuData {
public:
    uint8_t velocity_x;
    uint16_t acceleration_y;
};

} // namespace carmen_hardware

#endif /* COMMON_TYPES_H_ */
