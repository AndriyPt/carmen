#ifndef CARMEN_HARDWARE_PROTOCOL_H
#define CARMEN_HARDWARE_PROTOCOL_H

#include <orion_protocol/orion_header.h>
#include <stdint.h>

namespace carmen_hardware
{

enum class MessageType: uint8_t { Handshake = 1, ReadSettings = 2 };

#pragma pack(push, 1)

struct HandshakeCommand
{
  orion::CommandHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::Handshake),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0
    }
  };
};

struct HandshakeResult
{
  orion::ResultHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::Handshake),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0,
    },
    .error_code = 0
  };
};

struct ReadSettings
{
  int32_t left_front_p; // multiplied on 10000 and rounded
  int32_t left_front_i; // multiplied on 10000 and rounded
  int32_t left_front_d; // multiplied on 10000 and rounded
};

struct ReadSettingsCommand
{
  orion::CommandHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::ReadSettings),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0
    }
  };
  ReadSettings data;
};

struct ReadSettingsResult
{
  orion::ResultHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::ReadSettings),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0,
    },
    .error_code = 0
  };
  ReadSettings data;
};

#pragma pack(pop)

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_PROTOCOL_H
