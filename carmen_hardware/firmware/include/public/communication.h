#include "communication_base.h"

namespace carmen_hardware 
{

class Communication : public CommunicationBase 
{
protected:
    application::ImuData imu;
    encoder_t[] encoders;

protected:
    virtual void setImuHandler();
    virtual void setEncodersHandler();

public:
    bool setImu(ImuData* data);
    Communication() = delete;
    Communication& operator=(const Communication& object) = delete;
    Communication(const Communication& object) = delete;
};

} // namespace carmen_hardware
