#ifndef COMMON_SIGNALS_H_
#define COMMON_SIGNALS_H_

#include "qpcpp.h"

namespace carmen_hardware 
{

enum Signal
{
   COM_SET_IMU_SIG = Q_USER_SIG,
   COM_SET_ENCODERS_SIG,
   COM_COMMAND_SIG,
   BL_SET_MOTORS_SPEED_SIG,
   BL_SET_MOTORS_PID_SIG
};

} // namespace carmen_hardware

#endif /* COMMON_SIGNALS_H_ */
