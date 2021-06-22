#include "communication.h"
#include "osal_communication.h"
#include "log.h"
#include "error.h"
#include "control_loop.h"
#include "orion_protocol/orion_communication.hpp"
#include "orion_protocol/orion_transport.hpp"
#include "orion_protocol/orion_header.hpp"
#include "orion_protocol/orion_minor.hpp"
#include "carmen_hardware/protocol.h"

#define QUEUE_SIZE (10)
#define COMMAND_BUFFER_SIZE (1024)
#define COMMAND_RECEIVE_TIMEOUT (100)
#define READ_EVENT_TIMEOUT (100)
typedef enum
{
    EVT_COMMAND_RECEIVED = 1,
    EVT_SET_PID_RESULT_RECEIVED,
    EVT_SET_COMMANDS_RESULT_RECEIVED
} event_type_t;

typedef struct
{
    osal_communication_message_t current_message;
    uint16_t current_sequence_id;
    uint8_t command_buffer[COMMAND_BUFFER_SIZE];
    size_t command_size;
} context_t;

static context_t context;

static orion::Communication com_port;
static orion::Transport frame_transport = orion::Transport(&com_port);
static orion::Minor minor(&frame_transport);

static void loop_function(void);
static void process_handshake_receive(void);
static void process_set_commands_receive(void);
static void reply_set_commands(void);
static void process_set_pid_receive(void);
static void reply_set_pid(void);

void send_new_command_event(void)
{
    osal_communication_message_t queue_message;
    queue_message.event = EVT_COMMAND_RECEIVED;
    osal_communication_queue_put(&queue_message);
}

void communication_init(void)
{
    LOG_DEBUG("Communication thread initializing...");
    osal_communication_create_thread(loop_function, QUEUE_SIZE);
}

void loop_function(void)
{
    osal_communication_status_t status = osal_communication_queue_get(&context.current_message, READ_EVENT_TIMEOUT);
    if (OSAL_COM_STATUS_OK == status)
    {
    	ssize_t command_result = 0;
        switch (context.current_message.event)
        {
        case EVT_COMMAND_RECEIVED:
        	command_result = minor.receiveCommand(context.command_buffer, COMMAND_BUFFER_SIZE);
            if (command_result > 0)
            {
                context.command_size = command_result;
                SOFTWARE_ASSERT(NULL != context.command_buffer);
                SOFTWARE_ASSERT(context.command_size >= sizeof(orion::CommandHeader));
                orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(context.command_buffer);
                context.current_sequence_id = command_header->common.sequence_id;
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
            break;
        case EVT_SET_COMMANDS_RESULT_RECEIVED:
            reply_set_commands();
            break;

        case EVT_SET_PID_RESULT_RECEIVED:
            reply_set_pid();
            break;

        default:
            LOG_ERROR("Not supported event type");
            SOFTWARE_ASSERT(false);
        }
    }
}

void process_handshake_receive(void)
{
    SOFTWARE_ASSERT(NULL != context.command_buffer);
    SOFTWARE_ASSERT(context.command_size >= sizeof(orion::CommandHeader));
    orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(context.command_buffer);
    SOFTWARE_ASSERT(carmen_hardware::MessageType::Handshake ==
            (carmen_hardware::MessageType)command_header->common.message_id);
    carmen_hardware::HandshakeResult handshake_result;
    handshake_result.header.common.sequence_id = command_header->common.sequence_id;
    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    minor.sendResult((uint8_t*)&handshake_result, sizeof(handshake_result));
}

static void set_commands_callback(uint16_t sequence_id, bool result)
{
    if (context.current_sequence_id == sequence_id)
    {
        osal_communication_message_t message;
        message.data[0] = (uint8_t)result;
        message.event = EVT_SET_COMMANDS_RESULT_RECEIVED;
        message.sequence_id = sequence_id;
        osal_communication_queue_put(&message);
    }
}

void process_set_commands_receive(void)
{
    SOFTWARE_ASSERT(NULL != context.command_buffer);
    SOFTWARE_ASSERT(context.command_size >= sizeof(carmen_hardware::SetCommandsCommand));
    carmen_hardware::SetCommandsCommand * command =
            reinterpret_cast<carmen_hardware::SetCommandsCommand*>(context.command_buffer);
    SOFTWARE_ASSERT(carmen_hardware::MessageType::SetCommands ==
            (carmen_hardware::MessageType)command->header.common.message_id);

    control_loop_set_command_t message;
    message.left_cmd = command->left_cmd;
    message.right_cmd = command->right_cmd;
    message.sequence_id = command->header.common.sequence_id;
    message.result_callback = set_commands_callback;
    control_loop_set_commands(&message);
}

void reply_set_commands(void)
{
    if (context.current_sequence_id == context.current_message.sequence_id)
    {
        carmen_hardware::SetCommandsResult reply;
        reply.header.common.sequence_id = context.current_message.sequence_id;
        if (context.current_message.data[0])
        {
        	reply.wheel_pos_left = 100;
        }
        else
        {
        	reply.wheel_pos_left = 0;
        }
        // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
        minor.sendResult((uint8_t*)&reply, sizeof(reply));
    }
}

static void set_pid_callback(uint16_t sequence_id, bool result)
{
    if (context.current_sequence_id == sequence_id)
    {
        osal_communication_message_t message;
        message.data[0] = (uint8_t)result;
        message.event = EVT_SET_PID_RESULT_RECEIVED;
        message.sequence_id = sequence_id;
        osal_communication_queue_put(&message);
    }
}

void process_set_pid_receive(void)
{
    SOFTWARE_ASSERT(NULL != context.command_buffer);
    SOFTWARE_ASSERT(context.command_size >= sizeof(carmen_hardware::SetPIDCommand));
    carmen_hardware::SetPIDCommand * command =
            reinterpret_cast<carmen_hardware::SetPIDCommand*>(context.command_buffer);
    SOFTWARE_ASSERT(carmen_hardware::MessageType::SetPID ==
            (carmen_hardware::MessageType)command->header.common.message_id);

    control_loop_set_pid_t message;
    message.left_p = command->left_p;
    message.left_i = command->left_i;
    message.left_d = command->left_d;
    message.right_p = command->right_p;
    message.right_i = command->right_i;
    message.right_d = command->right_d;
    message.sequence_id = command->header.common.sequence_id;
    message.result_callback = set_pid_callback;
    control_loop_set_pid(&message);
}

void reply_set_pid(void)
{
    if (context.current_sequence_id == context.current_message.sequence_id)
    {
        carmen_hardware::SetPIDResult reply;
        reply.header.common.sequence_id = context.current_message.sequence_id;
        reply.result = (bool)context.current_message.data[0];
        // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
        minor.sendResult((uint8_t*)&reply, sizeof(reply));
    }
}

