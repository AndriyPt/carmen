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
      typedef float _left_motor_position_type;
      _left_motor_position_type left_motor_position;
      typedef float _left_motor_velocity_type;
      _left_motor_velocity_type left_motor_velocity;
      typedef float _right_motor_position_type;
      _right_motor_position_type right_motor_position;
      typedef float _right_motor_velocity_type;
      _right_motor_velocity_type right_motor_velocity;

    FirmwareStateRead():
      left_motor_position(0),
      left_motor_velocity(0),
      right_motor_position(0),
      right_motor_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor_position;
      u_left_motor_position.real = this->left_motor_position;
      *(outbuffer + offset + 0) = (u_left_motor_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_position);
      union {
        float real;
        uint32_t base;
      } u_left_motor_velocity;
      u_left_motor_velocity.real = this->left_motor_velocity;
      *(outbuffer + offset + 0) = (u_left_motor_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_velocity);
      union {
        float real;
        uint32_t base;
      } u_right_motor_position;
      u_right_motor_position.real = this->right_motor_position;
      *(outbuffer + offset + 0) = (u_right_motor_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_position);
      union {
        float real;
        uint32_t base;
      } u_right_motor_velocity;
      u_right_motor_velocity.real = this->right_motor_velocity;
      *(outbuffer + offset + 0) = (u_right_motor_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor_position;
      u_left_motor_position.base = 0;
      u_left_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_position = u_left_motor_position.real;
      offset += sizeof(this->left_motor_position);
      union {
        float real;
        uint32_t base;
      } u_left_motor_velocity;
      u_left_motor_velocity.base = 0;
      u_left_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_velocity = u_left_motor_velocity.real;
      offset += sizeof(this->left_motor_velocity);
      union {
        float real;
        uint32_t base;
      } u_right_motor_position;
      u_right_motor_position.base = 0;
      u_right_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_position = u_right_motor_position.real;
      offset += sizeof(this->right_motor_position);
      union {
        float real;
        uint32_t base;
      } u_right_motor_velocity;
      u_right_motor_velocity.base = 0;
      u_right_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_velocity = u_right_motor_velocity.real;
      offset += sizeof(this->right_motor_velocity);
     return offset;
    }

    const char * getType(){ return "carmen_msgs/FirmwareStateRead"; };
    const char * getMD5(){ return "4d3b29390e26cf7662818ef660ec042d"; };

  };

}
#endif