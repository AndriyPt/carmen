#include "communication_events.h"
#include "error.h"

namespace carmen_hardware 
{

SetImuEvt* SetImuEvt::create(Signal signal)
{
    SOFTWARE_ASSERT(COM_SET_IMU_SIG == signal);
    return Q_NEW(SetImuEvt, signal);    
}

};
