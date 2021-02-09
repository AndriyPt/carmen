#ifndef COMMON_SIGNALS_H_
#define COMMON_SIGNALS_H_

#include "qpcpp.h"

namespace carmen_hardware 
{

enum Signal
{
   BL_SET_IMU_SIG = Q_USER_SIG,
   BL_SET_ENCODERS_SIG,
   BL_COMMAND_SIG
};

} // namespace carmen_hardware

#endif /* COMMON_SIGNALS_H_ */
