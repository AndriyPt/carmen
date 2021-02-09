#include "business_logic_base.h"
#include "common_types.h"

namespace carmen_hardware 
{

class BusinessLogic : public BusinessLogicBase
{
public:

    BusinessLogic(orion::Minor *minor);
    BusinessLogic() = delete;
    BusinessLogic& operator=(const BusinessLogic& object) = delete;
    BusinessLogic(const BusinessLogic& object) = delete;

    void setImuData(double alpha, double beta, double gamma);

    void setEncoders(double left, double right);

protected:
    virtual void setImuHandler(SetImuEvt const* event);
    virtual void setEncodersHandler();
    virtual void commandHandler();

private:
    orion::Minor *minor_ = NULL;
    ImuData imu_;
    context_t context_;
    // encoder_t[] encoders;
};

} // namespace carmen_hardware
