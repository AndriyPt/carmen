#ifndef _ROS_carmen_msgs_FirmwareStateRead_h
#define _ROS_carmen_msgs_FirmwareStateRead_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace carmen_msgs
{

  class FirmwareStateRead : public ros::Msg
  {
    public:
      typedef float _motor_1_position_type;
      _motor_1_position_type motor_1_position;
      typedef float _motor_1_velocity_type;
      _motor_1_velocity_type motor_1_velocity;
      typedef float _motor_1_effort_type;
      _motor_1_effort_type motor_1_effort;
      typedef float _motor_2_position_type;
      _motor_2_position_type motor_2_position;
      typedef float _motor_2_velocity_type;
      _motor_2_velocity_type motor_2_velocity;
      typedef float _motor_2_effort_type;
      _motor_2_effort_type motor_2_effort;
      typedef float _motor_3_position_type;
      _motor_3_position_type motor_3_position;
      typedef float _motor_3_velocity_type;
      _motor_3_velocity_type motor_3_velocity;
      typedef float _motor_3_effort_type;
      _motor_3_effort_type motor_3_effort;
      typedef float _motor_4_position_type;
      _motor_4_position_type motor_4_position;
      typedef float _motor_4_velocity_type;
      _motor_4_velocity_type motor_4_velocity;
      typedef float _motor_4_effort_type;
      _motor_4_effort_type motor_4_effort;

    FirmwareStateRead():
      motor_1_position(0),
      motor_1_velocity(0),
      motor_1_effort(0),
      motor_2_position(0),
      motor_2_velocity(0),
      motor_2_effort(0),
      motor_3_position(0),
      motor_3_velocity(0),
      motor_3_effort(0),
      motor_4_position(0),
      motor_4_velocity(0),
      motor_4_effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motor_1_position;
      u_motor_1_position.real = this->motor_1_position;
      *(outbuffer + offset + 0) = (u_motor_1_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_1_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_1_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_1_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_1_position);
      union {
        float real;
        uint32_t base;
      } u_motor_1_velocity;
      u_motor_1_velocity.real = this->motor_1_velocity;
      *(outbuffer + offset + 0) = (u_motor_1_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_1_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_1_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_1_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_1_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_1_effort;
      u_motor_1_effort.real = this->motor_1_effort;
      *(outbuffer + offset + 0) = (u_motor_1_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_1_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_1_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_1_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_1_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_2_position;
      u_motor_2_position.real = this->motor_2_position;
      *(outbuffer + offset + 0) = (u_motor_2_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_2_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_2_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_2_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_2_position);
      union {
        float real;
        uint32_t base;
      } u_motor_2_velocity;
      u_motor_2_velocity.real = this->motor_2_velocity;
      *(outbuffer + offset + 0) = (u_motor_2_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_2_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_2_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_2_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_2_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_2_effort;
      u_motor_2_effort.real = this->motor_2_effort;
      *(outbuffer + offset + 0) = (u_motor_2_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_2_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_2_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_2_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_2_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_3_position;
      u_motor_3_position.real = this->motor_3_position;
      *(outbuffer + offset + 0) = (u_motor_3_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_3_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_3_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_3_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_3_position);
      union {
        float real;
        uint32_t base;
      } u_motor_3_velocity;
      u_motor_3_velocity.real = this->motor_3_velocity;
      *(outbuffer + offset + 0) = (u_motor_3_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_3_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_3_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_3_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_3_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_3_effort;
      u_motor_3_effort.real = this->motor_3_effort;
      *(outbuffer + offset + 0) = (u_motor_3_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_3_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_3_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_3_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_3_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_4_position;
      u_motor_4_position.real = this->motor_4_position;
      *(outbuffer + offset + 0) = (u_motor_4_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_4_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_4_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_4_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_4_position);
      union {
        float real;
        uint32_t base;
      } u_motor_4_velocity;
      u_motor_4_velocity.real = this->motor_4_velocity;
      *(outbuffer + offset + 0) = (u_motor_4_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_4_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_4_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_4_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_4_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_4_effort;
      u_motor_4_effort.real = this->motor_4_effort;
      *(outbuffer + offset + 0) = (u_motor_4_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_4_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_4_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_4_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_4_effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motor_1_position;
      u_motor_1_position.base = 0;
      u_motor_1_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_1_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_1_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_1_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_1_position = u_motor_1_position.real;
      offset += sizeof(this->motor_1_position);
      union {
        float real;
        uint32_t base;
      } u_motor_1_velocity;
      u_motor_1_velocity.base = 0;
      u_motor_1_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_1_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_1_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_1_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_1_velocity = u_motor_1_velocity.real;
      offset += sizeof(this->motor_1_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_1_effort;
      u_motor_1_effort.base = 0;
      u_motor_1_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_1_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_1_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_1_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_1_effort = u_motor_1_effort.real;
      offset += sizeof(this->motor_1_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_2_position;
      u_motor_2_position.base = 0;
      u_motor_2_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_2_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_2_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_2_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_2_position = u_motor_2_position.real;
      offset += sizeof(this->motor_2_position);
      union {
        float real;
        uint32_t base;
      } u_motor_2_velocity;
      u_motor_2_velocity.base = 0;
      u_motor_2_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_2_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_2_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_2_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_2_velocity = u_motor_2_velocity.real;
      offset += sizeof(this->motor_2_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_2_effort;
      u_motor_2_effort.base = 0;
      u_motor_2_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_2_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_2_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_2_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_2_effort = u_motor_2_effort.real;
      offset += sizeof(this->motor_2_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_3_position;
      u_motor_3_position.base = 0;
      u_motor_3_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_3_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_3_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_3_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_3_position = u_motor_3_position.real;
      offset += sizeof(this->motor_3_position);
      union {
        float real;
        uint32_t base;
      } u_motor_3_velocity;
      u_motor_3_velocity.base = 0;
      u_motor_3_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_3_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_3_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_3_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_3_velocity = u_motor_3_velocity.real;
      offset += sizeof(this->motor_3_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_3_effort;
      u_motor_3_effort.base = 0;
      u_motor_3_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_3_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_3_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_3_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_3_effort = u_motor_3_effort.real;
      offset += sizeof(this->motor_3_effort);
      union {
        float real;
        uint32_t base;
      } u_motor_4_position;
      u_motor_4_position.base = 0;
      u_motor_4_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_4_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_4_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_4_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_4_position = u_motor_4_position.real;
      offset += sizeof(this->motor_4_position);
      union {
        float real;
        uint32_t base;
      } u_motor_4_velocity;
      u_motor_4_velocity.base = 0;
      u_motor_4_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_4_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_4_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_4_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_4_velocity = u_motor_4_velocity.real;
      offset += sizeof(this->motor_4_velocity);
      union {
        float real;
        uint32_t base;
      } u_motor_4_effort;
      u_motor_4_effort.base = 0;
      u_motor_4_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_4_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_4_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_4_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_4_effort = u_motor_4_effort.real;
      offset += sizeof(this->motor_4_effort);
     return offset;
    }

    const char * getType(){ return "carmen_msgs/FirmwareStateRead"; };
    const char * getMD5(){ return "83e06b6e62d184f88b7ac61f4a8ae753"; };

  };

}
#endif