#include "osal_control_loop.h"
#include "cmsis_os2.h"

static osMessageQueueId_t message_queue_id = NULL;
static osThreadId_t thread_id = NULL;

static const osThreadAttr_t task_attributes = { .name = "ControlLoopTask", .priority = (osPriority_t)osPriorityNormal,
        .stack_size = 5 * 1024 };

static void osal_thread_function(void *argument);

void osal_thread_function(void *argument)
{
    if (NULL != argument)
    {
        for (;;)
        {
            ((osal_control_loop_function_t)argument)();
            osDelay(500);
        }
    }
}

void osal_control_loop_create_thread(osal_control_loop_function_t fp_thread_function, uint16_t queue_size,
        uint16_t message_size)
{
    thread_id = osThreadNew(osal_thread_function, (void*)fp_thread_function, &task_attributes);

    message_queue_id = osMessageQueueNew(queue_size, message_size, NULL);
}

void osal_control_loop_queue_put(const void *p_message)
{
    //TODO: Add error checking of return code
    osMessageQueuePut(message_queue_id, p_message, 1, 100);
}

bool osal_control_loop_queue_get(void *p_message, uint32_t timeout)
{
    //TODO: Add error checking of return code
    uint8_t msg_prio = 0;
    osStatus_t result = osMessageQueueGet(message_queue_id, p_message, &msg_prio, timeout);
    if (osOK == result)
    {
        return true;
    }
    return false;
}
