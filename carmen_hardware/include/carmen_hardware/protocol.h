#ifndef CARMEN_HARDWARE_PROTOCOL_H
#define CARMEN_HARDWARE_PROTOCOL_H

#include <orion_protocol/orion_header.h>
#include <stdint.h>
#include <stdbool.h>

// ROS Units https://www.ros.org/reps/rep-0103.html

namespace carmen_hardware
{

enum class ErrorType: uint8_t { TimeoutExpired = 1 };

enum class MessageType: uint8_t { Handshake = 1, ReadSettings = 2, SetCommands = 3, SetPID = 4 };

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
  int32_t left_p; // multiplied by 10000 and rounded
  int32_t left_i; // multiplied by 10000 and rounded
  int32_t left_d; // multiplied by 10000 and rounded

  int32_t right_p; // multiplied by 10000 and rounded
  int32_t right_i; // multiplied by 10000 and rounded
  int32_t right_d; // multiplied by 10000 and rounded
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

struct SetCommandsCommand
{
  orion::CommandHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::SetCommands),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0
    }
  };
  int16_t left_cmd; // multiplied by 1000 and rounded
  int16_t right_cmd; // multiplied by 1000 and rounded
};

struct SetCommandsResult
{
  orion::ResultHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::SetCommands),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0,
    },
    .error_code = 0
  };
  int32_t encoder_left; // multiplied by 1000 and rounded
  int32_t encoder_right; // multiplied by 1000 and rounded
  int32_t imu_angle_alpha; // multiplied by 1000 and rounded
  int32_t imu_angle_beta; // multiplied by 1000 and rounded
  int32_t imu_angle_gamma; // multiplied by 1000 and rounded
};

struct SetPIDCommand
{
  orion::CommandHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::SetPID),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0
    }
  };
  uint16_t left_p;
  uint16_t left_i;
  uint16_t left_d;
  uint16_t right_p;
  uint16_t right_i;
  uint16_t right_d;
};

struct SetPIDResult
{
  orion::ResultHeader header =
  {
    .frame =
    {
      .crc = 0
    },
    .common =
    {
      .message_id = static_cast<uint8_t>(MessageType::SetPID),
      .version = 1,
      .oldest_compatible_version = 1,
      .sequence_id = 0,
    },
    .error_code = 0
  };
  bool result;
};

#pragma pack(pop)

}  // carmen_hardware

#endif  // CARMEN_HARDWARE_PROTOCOL_H
