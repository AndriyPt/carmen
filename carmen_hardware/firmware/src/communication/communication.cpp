#include "communication.h"
#include "qpcpp.h"
#include "communication_events.h"
#include "error.h"

namespace carmen_hardware
{

Communication::Communication(orion::Minor *minor, QActive *commands_executor): CommunicationBase(), 
    minor_(minor), commands_executor_(commands_executor)
{

}

void Communication::setImuHandler(SetImuEvt const* event)
{
    SOFTWARE_ASSERT(NULL != event);
    this->imu_.velocity_x = event->data.velocity_x; 
    this->imu_.acceleration_y = event->data.acceleration_y; 
}

void Communication::setEncodersHandler()
{

}


}