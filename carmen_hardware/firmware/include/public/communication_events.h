#ifndef COMMUNICATION_EVENTS_H_
#define COMMUNICATION_EVENTS_H_

#include "qpcpp.h"
#include "common_types.h"
#include "common_signals.h"

namespace carmen_hardware 
{

class SetImuEvt : public QP::QEvt 
{
public:
    ImuData data;

    static SetImuEvt* create(Signal signal);
};

} // namespace carmen_hardware

#endif /* COMMUNICATION_EVENTS_H_ */
