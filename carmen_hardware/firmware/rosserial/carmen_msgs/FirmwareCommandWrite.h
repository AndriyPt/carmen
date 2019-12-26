#ifndef _ROS_carmen_msgs_FirmwareCommandWrite_h
#define _ROS_carmen_msgs_FirmwareCommandWrite_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace carmen_msgs
{

  class FirmwareCommandWrite : public ros::Msg
  {
    public:
      typedef float _left_motor_p_type;
      _left_motor_p_type left_motor_p;
      typedef float _left_motor_i_type;
      _left_motor_i_type left_motor_i;
      typedef float _left_motor_d_type;
      _left_motor_d_type left_motor_d;
      typedef float _right_motor_p_type;
      _right_motor_p_type right_motor_p;
      typedef float _right_motor_i_type;
      _right_motor_i_type right_motor_i;
      typedef float _right_motor_d_type;
      _right_motor_d_type right_motor_d;
      typedef float _left_motor_velocity_command_type;
      _left_motor_velocity_command_type left_motor_velocity_command;
      typedef float _right_motor_velocity_command_type;
      _right_motor_velocity_command_type right_motor_velocity_command;

    FirmwareCommandWrite():
      left_motor_p(0),
      left_motor_i(0),
      left_motor_d(0),
      right_motor_p(0),
      right_motor_i(0),
      right_motor_d(0),
      left_motor_velocity_command(0),
      right_motor_velocity_command(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor_p;
      u_left_motor_p.real = this->left_motor_p;
      *(outbuffer + offset + 0) = (u_left_motor_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_p);
      union {
        float real;
        uint32_t base;
      } u_left_motor_i;
      u_left_motor_i.real = this->left_motor_i;
      *(outbuffer + offset + 0) = (u_left_motor_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_i);
      union {
        float real;
        uint32_t base;
      } u_left_motor_d;
      u_left_motor_d.real = this->left_motor_d;
      *(outbuffer + offset + 0) = (u_left_motor_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_d);
      union {
        float real;
        uint32_t base;
      } u_right_motor_p;
      u_right_motor_p.real = this->right_motor_p;
      *(outbuffer + offset + 0) = (u_right_motor_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_p);
      union {
        float real;
        uint32_t base;
      } u_right_motor_i;
      u_right_motor_i.real = this->right_motor_i;
      *(outbuffer + offset + 0) = (u_right_motor_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_i);
      union {
        float real;
        uint32_t base;
      } u_right_motor_d;
      u_right_motor_d.real = this->right_motor_d;
      *(outbuffer + offset + 0) = (u_right_motor_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_d);
      union {
        float real;
        uint32_t base;
      } u_left_motor_velocity_command;
      u_left_motor_velocity_command.real = this->left_motor_velocity_command;
      *(outbuffer + offset + 0) = (u_left_motor_velocity_command.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_velocity_command.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_velocity_command.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_velocity_command.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_velocity_command);
      union {
        float real;
        uint32_t base;
      } u_right_motor_velocity_command;
      u_right_motor_velocity_command.real = this->right_motor_velocity_command;
      *(outbuffer + offset + 0) = (u_right_motor_velocity_command.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_velocity_command.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_velocity_command.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_velocity_command.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_velocity_command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor_p;
      u_left_motor_p.base = 0;
      u_left_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_p = u_left_motor_p.real;
      offset += sizeof(this->left_motor_p);
      union {
        float real;
        uint32_t base;
      } u_left_motor_i;
      u_left_motor_i.base = 0;
      u_left_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_i = u_left_motor_i.real;
      offset += sizeof(this->left_motor_i);
      union {
        float real;
        uint32_t base;
      } u_left_motor_d;
      u_left_motor_d.base = 0;
      u_left_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_d = u_left_motor_d.real;
      offset += sizeof(this->left_motor_d);
      union {
        float real;
        uint32_t base;
      } u_right_motor_p;
      u_right_motor_p.base = 0;
      u_right_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_p = u_right_motor_p.real;
      offset += sizeof(this->right_motor_p);
      union {
        float real;
        uint32_t base;
      } u_right_motor_i;
      u_right_motor_i.base = 0;
      u_right_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_i = u_right_motor_i.real;
      offset += sizeof(this->right_motor_i);
      union {
        float real;
        uint32_t base;
      } u_right_motor_d;
      u_right_motor_d.base = 0;
      u_right_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_d = u_right_motor_d.real;
      offset += sizeof(this->right_motor_d);
      union {
        float real;
        uint32_t base;
      } u_left_motor_velocity_command;
      u_left_motor_velocity_command.base = 0;
      u_left_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_velocity_command = u_left_motor_velocity_command.real;
      offset += sizeof(this->left_motor_velocity_command);
      union {
        float real;
        uint32_t base;
      } u_right_motor_velocity_command;
      u_right_motor_velocity_command.base = 0;
      u_right_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_velocity_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_velocity_command = u_right_motor_velocity_command.real;
      offset += sizeof(this->right_motor_velocity_command);
     return offset;
    }

    const char * getType(){ return "carmen_msgs/FirmwareCommandWrite"; };
    const char * getMD5(){ return "d9ce22517fbab8435fa14c41f2ff63d6"; };

  };

}
#endif