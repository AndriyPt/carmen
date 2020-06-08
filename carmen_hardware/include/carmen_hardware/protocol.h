#ifndef CARMEN_HARDWARE_PROTOCOL_H
#define CARMEN_HARDWARE_PROTOCOL_H

namespace carmen_hardware
{

#pragma pack(push, 1)

struct ReadSettings
{
  int32_t left_front_p; // multiplied on 10000 and rounded
  int32_t left_front_i; // multiplied on 10000 and rounded
  int32_t left_front_d; // multiplied on 10000 and rounded
};

struct ReadSettingsCommand
{
  ReadSettings data;
};

struct ReadSettingsResult
{
  ReadSettings data;
};

#pragma pack(pop)

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_PROTOCOL_H
