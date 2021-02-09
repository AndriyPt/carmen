#include "communication.h"
#include "qpcpp.h"
#include "communication_events.h"
#include "error.h"
#include "carmen_hardware/protocol.h"

namespace carmen_hardware
{

class SetImuEvt : public QP::QEvt 
{
public:
    ImuData data;
};

Communication::Communication(orion::Minor *minor, QActive *commands_executor): CommunicationBase(), 
    minor_(minor), commands_executor_(commands_executor)
{
}

void Communication::sendNewCommandEvent()
{
    return Q_NEW(SetImuEvt, signal);    

}

void Communication::process_handshake_receive(void)
{
    SOFTWARE_ASSERT(NULL != this->context_.command_buffer);
    SOFTWARE_ASSERT(this->context_.command_size >= sizeof(orion::CommandHeader));
    orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->context_.command_buffer);
    SOFTWARE_ASSERT(carmen_hardware::MessageType::Handshake ==
            (carmen_hardware::MessageType)command_header->common.message_id);
    carmen_hardware::HandshakeResult handshake_result;
    handshake_result.header.common.sequence_id = command_header->common.sequence_id;
    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&handshake_result, sizeof(handshake_result));
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

void Communication::commandHandler()
{
    if (this->minor_.receiveCommand(this->context_.command_buffer, COMMAND_BUFFER_SIZE, this->context_.command_size))
    {
        SOFTWARE_ASSERT(NULL != this->context_.command_buffer);
        SOFTWARE_ASSERT(this->context_.command_size >= sizeof(orion::CommandHeader));
        orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->context_.command_buffer);
        this->context_.current_sequence_id = command_header->common.sequence_id;
        switch (static_cast<carmen_hardware::MessageType>(command_header->common.message_id))
        {
            case carmen_hardware::MessageType::Handshake:
                process_handshake_receive();
                break;
            case carmen_hardware::MessageType::SetCommands:
                process_set_commands_receive();
                break;
            case carmen_hardware::MessageType::SetPID:
                process_set_pid_receive();
                break;
            default:
                LOG_ERROR("Not supported command type");
                SOFTWARE_ASSERT(false);
        }
    }
}

}