#include "communication.h"
#include "qpcpp.h"

namespace carmen_hardware
{

bool Communication::setImu(ImuData* data)
{
    SetImuEvt *pe = Q_NEW(SetImuEvt, SET_IMU_SIG);
    pe->data.velocity_x = data->velocity_x;
    pe->data.acceleration_y = data->acceleration_y;
    QF_PUBLISH(&pe->super, this);
}

void Communication::setImuHandler()
{

}

void Communication::setEncodersHandler()
{

}


}