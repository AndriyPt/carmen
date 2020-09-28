#include "control_loop.h"
#include "osal_control_loop.h"
#include "log.h"
#include "error.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define QUEUE_SIZE (10)

static void loop_function(void);

typedef enum
{
    MSG_SET_COMMANDS,
    MSG_SET_PID
} message_type_t;

typedef struct
{
    osal_cl_message_t current_message;
} state_t;

static state_t state;

void loop_function(void)
{
    if (OSAL_CL_STATUS_OK != osal_control_loop_queue_get(&state.current_message, 100))
    {
        return;
    }

    control_loop_set_command_t * info;
    control_loop_set_pid_t * pid_info;

    switch (state.current_message.message_type)
    {
    case MSG_SET_COMMANDS:
        info = (control_loop_set_command_t*)state.current_message.data;
        if (NULL != info->result_callback)
        {
            info->result_callback(info->sequence_id, true);
        }
        break;
    case MSG_SET_PID:
        pid_info = (control_loop_set_pid_t*)state.current_message.data;
        if (NULL != pid_info->result_callback)
        {
            pid_info->result_callback(pid_info->sequence_id, false);
        }
        break;
    }
}

void control_loop_init()
{
    LOG_DEBUG("Control loop thread initializing...");
    osal_control_loop_create_thread(loop_function, QUEUE_SIZE);
}

void control_loop_set_commands(const control_loop_set_command_t *p_message)
{
    SOFTWARE_ERROR(NULL == p_message);
    SOFTWARE_ERROR(OSAL_CL_MESSAGE_DATA_SIZE < sizeof(&p_message));

    osal_cl_message_t queue_message;
    queue_message.message_type = MSG_SET_COMMANDS;
    memcpy(queue_message.data, p_message, sizeof(*p_message));
    osal_control_loop_queue_put(&queue_message);
}

void control_loop_set_pid(const control_loop_set_pid_t *p_message)
{
    SOFTWARE_ERROR(NULL == p_message);
    SOFTWARE_ERROR(OSAL_CL_MESSAGE_DATA_SIZE < sizeof(&p_message));

    osal_cl_message_t queue_message;
    queue_message.message_type = MSG_SET_PID;
    memcpy(queue_message.data, p_message, sizeof(*p_message));
    osal_control_loop_queue_put(&queue_message);
}
