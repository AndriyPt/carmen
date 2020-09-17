#include "communication.h"
#include "osal_communication.h"
#include "log.h"
#include "error.h"
#include "bt.h"
#include "control_loop.h"
#include "orion_protocol/orion_header.h"
#include "orion_protocol/orion_minor.h"
#include "carmen_hardware/protocol.h"

#define QUEUE_SIZE (10)

using BrainTree::Node;

static void loop_function(void);

typedef enum
{
    MSG_COMMAND_RECEIVED = 1,
    MSG_SET_PID_REQUEST_RECEIVED,
    MSG_SET_PID_RESULT_RECEIVED,
    MSG_SET_COMMANDS_REQUEST_RECEIVED,
    MSG_SET_COMMANDS_RESULT_RECEIVED
} message_type_t;

#define COMMAND_BUFFER_SIZE (1024)
#define COMMAND_RECEIVE_TIMEOUT (100)

typedef struct
{
    osal_communication_message_t current_message;
    uint16_t current_sequence_id;
    uint8_t command_buffer[COMMAND_BUFFER_SIZE];
    size_t command_size;
} state_t;

static state_t state;
static orion::Minor minor;

static BrainTree::BehaviorTree behaviour_tree;

typedef Node::Status (*action_function_t)();

class FunctionalAction : public BrainTree::Node
{
public:
    FunctionalAction(const action_function_t function) : m_function(function) {}

    Status update() override
    {
        Status result = m_function();
        return (result);
    }

private:
    const action_function_t m_function;
};

static Node::Status read_queue_action()
{
    osal_communication_status_t status = osal_communication_queue_get(&state.current_message, 100);
    if (OSAL_COM_STATUS_OK == status)
    {
        return Node::Status::Success;
    }
    return Node::Status::Failure;
}

static Node::Status read_command_action()
{
    if (MSG_COMMAND_RECEIVED == state.current_message.message_type)
    {
        if (minor.receiveCommand(state.command_buffer, COMMAND_BUFFER_SIZE, COMMAND_RECEIVE_TIMEOUT,
                state.command_size))
        {
            orion::CommandHeader *command_header = reinterpret_cast<orion::CommandHeader*>(state.command_buffer);
            state.current_sequence_id = command_header->common.sequence_id;
            return Node::Status::Success;
        }
    }
    return Node::Status::Failure;
}

static Node::Status process_handshake_receive_action()
{
    SOFTWARE_ERROR(NULL == state.command_buffer);
    orion::CommandHeader *command_header = reinterpret_cast<orion::CommandHeader*>(state.command_buffer);
    if (carmen_hardware::MessageType::Handshake == (carmen_hardware::MessageType)command_header->common.message_id)
    {
        carmen_hardware::HandshakeResult handshake_result;
        handshake_result.header.common.sequence_id = command_header->common.sequence_id;
//        TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
        minor.sendResult((uint8_t*)&handshake_result, sizeof(handshake_result));
        return Node::Status::Success;
    }
    return Node::Status::Failure;
}

static void set_commands_callback(uint16_t sequence_id, bool result)
{
    if (state.current_sequence_id == sequence_id)
    {
        osal_communication_message_t message;
        message.data[0] = (uint8_t)result;
        message.message_type = MSG_SET_COMMANDS_RESULT_RECEIVED;
        message.sequence_id = sequence_id;
        osal_communication_queue_put(&message);
    }
}

static Node::Status process_set_commands_receive_action()
{
    SOFTWARE_ERROR(NULL == state.command_buffer);
    SOFTWARE_ERROR(state.command_size < sizeof(orion::CommandHeader));
    orion::CommandHeader *command_header = reinterpret_cast<orion::CommandHeader*>(state.command_buffer);
    if (carmen_hardware::MessageType::SetCommands == (carmen_hardware::MessageType)command_header->common.message_id)
    {
        SOFTWARE_ERROR(state.command_size < sizeof(carmen_hardware::SetCommandsCommand));
        carmen_hardware::SetCommandsCommand *command = reinterpret_cast<carmen_hardware::SetCommandsCommand*>(
                state.command_buffer);

        control_loop_set_command_t message;
        message.left_cmd = command->left_cmd;
        message.right_cmd = command->right_cmd;
        message.sequence_id = command_header->common.sequence_id;
        message.result_callback = set_commands_callback;
        control_loop_set_commands(&message);
        return Node::Status::Success;
    }
    return Node::Status::Failure;
}

static Node::Status reply_set_commands_action()
{
    if (MSG_SET_COMMANDS_RESULT_RECEIVED == state.current_message.message_type)
    {
        if (state.current_sequence_id == state.current_message.sequence_id)
        {
            carmen_hardware::SetCommandsResult reply;
            reply.result = (bool)state.current_message.data[0];
            minor.sendResult((uint8_t*)&reply, sizeof(reply));
            return Node::Status::Success;
        }
    }
    return Node::Status::Failure;
}

static void set_pid_callback(uint16_t sequence_id, bool result)
{
    if (state.current_sequence_id == sequence_id)
    {
        osal_communication_message_t message;
        message.data[0] = (uint8_t)result;
        message.message_type = MSG_SET_PID_RESULT_RECEIVED;
        message.sequence_id = sequence_id;
        osal_communication_queue_put(&message);
    }
}

static Node::Status process_set_pid_receive_action()
{
    SOFTWARE_ERROR(NULL == state.command_buffer);
    SOFTWARE_ERROR(state.command_size < sizeof(orion::CommandHeader));
    orion::CommandHeader *command_header = reinterpret_cast<orion::CommandHeader*>(state.command_buffer);
    if (carmen_hardware::MessageType::SetPID == (carmen_hardware::MessageType)command_header->common.message_id)
    {
        SOFTWARE_ERROR(state.command_size < sizeof(carmen_hardware::SetCommandsCommand));
        carmen_hardware::SetPIDCommand *command = reinterpret_cast<carmen_hardware::SetPIDCommand*>(
                state.command_buffer);

        control_loop_set_pid_t message;
        message.left_p = command->left_p;
        message.left_i = command->left_i;
        message.left_d = command->left_d;
        message.right_p = command->right_p;
        message.right_i = command->right_i;
        message.right_d = command->right_d;
        message.sequence_id = command_header->common.sequence_id;
        message.result_callback = set_pid_callback;
        control_loop_set_pid(&message);
        return Node::Status::Success;
    }
    return Node::Status::Failure;
}

static Node::Status reply_set_pid_action()
{
    if (MSG_SET_PID_RESULT_RECEIVED == state.current_message.message_type)
    {
        if (state.current_sequence_id == state.current_message.sequence_id)
        {
            carmen_hardware::SetPIDResult reply;
            reply.result = (bool)state.current_message.data[0];
            minor.sendResult((uint8_t*)&reply, sizeof(reply));
            return Node::Status::Success;
        }
    }
    return Node::Status::Failure;
}

void loop_function(void)
{
    behaviour_tree.update();
}

void send_new_command_event(void)
{
    osal_communication_message_t queue_message;
    queue_message.message_type = MSG_COMMAND_RECEIVED;
    osal_communication_queue_put(&queue_message);
}

void communication_init(void)
{
    auto p_sequence_1 = std::make_shared<BrainTree::Sequence>();
    {
        auto p_read_queue_action = std::make_shared<FunctionalAction>(read_queue_action);
        p_sequence_1->addChild(p_read_queue_action);
        auto p_selector_1 = std::make_shared<BrainTree::Selector>();
        p_sequence_1->addChild(p_selector_1);
        {
            auto p_sequence_2 = std::make_shared<BrainTree::Sequence>();
            p_selector_1->addChild(p_sequence_2);
            {
                {
                    auto p_read_command_action = std::make_shared<FunctionalAction>(read_command_action);
                    p_sequence_2->addChild(p_read_command_action);
                }
                {
                    auto p_selector_2 = std::make_shared<BrainTree::Selector>();
                    p_sequence_2->addChild(p_selector_2);
                    {
                        auto p_process_handshake_receive_action = std::make_shared<FunctionalAction>(
                                process_handshake_receive_action);
                        p_selector_2->addChild(p_process_handshake_receive_action);
                        auto p_process_set_commands_receive_action = std::make_shared<FunctionalAction>(
                                process_set_commands_receive_action);
                        p_selector_2->addChild(p_process_set_commands_receive_action);
                        auto p_process_set_pid_receive_action = std::make_shared<FunctionalAction>(
                                process_set_pid_receive_action);
                        p_selector_2->addChild(p_process_set_pid_receive_action);
                    }
                }
            }
            auto p_reply_set_commands_action = std::make_shared<FunctionalAction>(
                    reply_set_commands_action);
            p_selector_1->addChild(p_reply_set_commands_action);
            auto p_reply_set_pid_action = std::make_shared<FunctionalAction>(reply_set_pid_action);
            p_selector_1->addChild(p_reply_set_pid_action);
        }
    }
    behaviour_tree.setRoot(p_sequence_1);

    LOG_DEBUG("Communication thread initializing...");
    osal_communication_create_thread(loop_function, QUEUE_SIZE);
}
