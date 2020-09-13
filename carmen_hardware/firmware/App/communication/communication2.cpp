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
    MSG_PID_RESULT_RECEIVED,
    MSG_COMMANDS_RESULT_RECEIVED
} message_type_t;

typedef struct
{
    osal_communication_message_t current_message;
    uint16_t current_sequence_id;
} state_t;

static state_t state;

static BrainTree::BehaviorTree tree;

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

void loop_function(void)
{

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
    tree.setRoot(sequence_1);

    LOG_DEBUG("Communication thread initializing...");
    osal_communication_create_thread(loop_function, QUEUE_SIZE);
}
