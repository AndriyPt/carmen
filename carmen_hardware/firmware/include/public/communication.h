#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "communication_base.h"
#include "orion_protocol/orion_minor.h"
#include "common_types.h"
#include "events.h"
#include "qpcpp.h"

namespace carmen_hardware 
{

class Communication : public CommunicationBase 
{
public:
    bool setImu(ImuData* data);
    Communication(orion::Minor *minor, QActive *commands_executor);
    Communication() = delete;
    Communication& operator=(const Communication& object) = delete;
    Communication(const Communication& object) = delete;

protected:
    virtual void setImuHandler(SetImuEvt const* event);
    virtual void setEncodersHandler();

private:
    orion::Minor *minor_ = NULL;
    QActive *commands_executor_ = NULL;
    ImuData imu_;
    // encoder_t[] encoders;
};

} // namespace carmen_hardware

#endif /* COMMUNICATION_H_ */
