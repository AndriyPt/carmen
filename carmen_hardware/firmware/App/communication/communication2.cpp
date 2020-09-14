#include "communication.h"
#include "osal_communication.h"
#include "log.h"
#include "error.h"
#include "bt.h"

#define QUEUE_SIZE (10)

static void loop_function(void);

typedef enum
{
    MSG_COMMAND_RECEIVED,
    MSG_PID_REQUEST_RECEIVED,
    MSG_PID_RESULT_RECEIVED,
    MSG_SET_COMMANDS_REQUEST_RECEIVED,
    MSG_COMMANDS_RESULT_RECEIVED
} message_type_t;

typedef struct
{
    osal_communication_message_t current_message;
    uint16_t current_sequence_id;
} state_t;

static state_t state;

static BrainTree::BehaviorTree behaviour_tree;

class StatefulAction : public BrainTree::Node
{
public:
    StatefulAction(state_t * state)
    {
        SOFTWARE_ERROR(NULL == state);
        m_state = state;
    }
protected:
    state_t * m_state;
};

class ReadQueueAction : public StatefulAction
{
public:
    ReadQueueAction(state_t * state) : StatefulAction(state)
    {
    }

    Status update() override
    {
        osal_communication_status_t status = osal_communication_queue_get(&m_state->current_message, 100);
        if (OSAL_COM_STATUS_OK == status)
        {
            return Node::Status::Success;
        }
        return Node::Status::Failure;
    }
};

class IsMessageTypeAction : public StatefulAction
{
public:
    IsMessageTypeAction(state_t * state, message_type_t message_type) : StatefulAction(state)
    {
        m_message_type = message_type;
    }

    Status update() override
    {
        if (m_state->current_message.message_type == m_message_type)
        {
            if (MSG_COMMAND_RECEIVED == m_message_type)
            {
                return Node::Status::Success;
            }
            if (m_state->current_message.sequence_id == m_state->current_sequence_id)
            {
                return Node::Status::Success;
            }
        }
        return Node::Status::Failure;
    }
private:
    message_type_t m_message_type;
};

class SendResultAction : public StatefulAction
{
public:
    SendResultAction(state_t * state, const uint8_t * buffer, size_t size) : StatefulAction(state)
    {
        SOFTWARE_ERROR(NULL == buffer);
        SOFTWARE_ERROR(0 == size);

        m_buffer = buffer;
        m_size = size;
    }

    Status update() override
    {
//        if (minor.send_reply(m_buffer, size))
//        {
//            return Node::Status::Success;
//        }
        return Node::Status::Failure;
    }

private:
    const uint8_t * m_buffer;
    size_t m_size;
};

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
    auto sequence_1 = std::make_shared<BrainTree::Sequence>();
    auto read_queue_action = std::make_shared<ReadQueueAction>(&state);
    auto selector_1 = std::make_shared<BrainTree::Selector>();
    sequence_1->addChild(read_queue_action);
    sequence_1->addChild(selector_1);
    auto sequence_2 = std::make_shared<BrainTree::Sequence>();
    selector_1->addChild(sequence_2);
    auto set_command_action = std::make_shared<IsMessageTypeAction>(&state, MSG_SET_COMMANDS_REQUEST_RECEIVED);
    sequence_2->addChild(set_command_action);
    //    auto send_command_reply_action = std::make_shared<SendResultAction>(&state, state-

    behaviour_tree.setRoot(sequence_1);

    LOG_DEBUG("Communication thread initializing...");
    osal_communication_create_thread(loop_function, QUEUE_SIZE);
}
