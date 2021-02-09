#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "communication_base.h"
#include "commands_executor.h"
#include "orion_protocol/orion_minor.h"
#include "common_types.h"
#include "events.h"
#include "qpcpp.h"

namespace carmen_hardware 
{

#define QUEUE_SIZE (10)
#define COMMAND_BUFFER_SIZE (1024)
#define COMMAND_RECEIVE_TIMEOUT (100)
#define READ_EVENT_TIMEOUT (100)

typedef struct
{
    osal_communication_message_t current_message;
    uint16_t current_sequence_id;
    uint8_t command_buffer[COMMAND_BUFFER_SIZE];
    size_t command_size;
} context_t;

class Communication : public CommunicationBase 
{
public:
    Communication(orion::Minor *minor, CommandsExecutor *commands_executor);
    Communication() = delete;
    Communication& operator=(const Communication& object) = delete;
    Communication(const Communication& object) = delete;

    bool setImu(ImuData* data);
    void sendNewCommandEvent();

protected:
    virtual void setImuHandler(SetImuEvt const* event);
    virtual void setEncodersHandler();
    virtual void commandHandler();

    void process_handshake_receive(void);

private:
    orion::Minor *minor_ = NULL;
    CommandsExecutor *commands_executor_ = NULL;
    ImuData imu_;
    context_t context_;
    // encoder_t[] encoders;
};

} // namespace carmen_hardware

#endif /* COMMUNICATION_H_ */
