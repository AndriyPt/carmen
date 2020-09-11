#include "communication.h"
#include "osal_communication.h"
#include "log.h"
#include "error.h"

#define QUEUE_SIZE (10)

static void loop_function(void);

typedef enum
{
    MSG_COMMAND_RECEIVED,
    MSG_PID_RESULT_RECEIVED,
    MSG_COMMANDS_RESULT_RECEIVED
} message_type_t;


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
    LOG_DEBUG("Communication thread initializing...");
    osal_communication_create_thread(loop_function, QUEUE_SIZE);
}
