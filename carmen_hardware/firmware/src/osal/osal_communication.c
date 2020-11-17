#include "osal_communication.h"
#include "cmsis_os2.h"
#include "error.h"

static osMessageQueueId_t message_queue_id = NULL;
static osThreadId_t thread_id = NULL;

static const osThreadAttr_t task_attributes = { .name = "CommunicationTask", .priority = (osPriority_t)osPriorityAboveNormal,
        .stack_size = 5 * 1024 };

static void osal_thread_function(void *argument);

void osal_thread_function(void *argument)
{
    if (NULL != argument)
    {
        for (;;)
        {
            ((osal_communication_function_t)argument)();
            osDelay(100);
        }
    }
}

void osal_communication_create_thread(osal_communication_function_t fp_thread_function, uint16_t queue_size)
{
    thread_id = osThreadNew(osal_thread_function, (void*)fp_thread_function, &task_attributes);
    message_queue_id = osMessageQueueNew(queue_size, sizeof(osal_communication_message_t), NULL);
}

osal_communication_status_t osal_communication_queue_put(const osal_communication_message_t *p_message)
{
    osal_communication_status_t result = OSAL_COM_STATUS_OK;
    //TODO: Think how to handle ISR timeout limitation in queue put
    osStatus_t status = osMessageQueuePut(message_queue_id, p_message, 1, 0);
    if (osOK != status)
    {
        SOFTWARE_ASSERT(osErrorTimeout == status);
        result = OSAL_COM_STATUS_TIMEOUT;
    }
    return (result);
}

osal_communication_status_t osal_communication_queue_get(osal_communication_message_t *p_message, uint32_t timeout)
{
    osal_communication_status_t result = OSAL_COM_STATUS_OK;
    uint8_t msg_prio = 0;
    osStatus_t status = osMessageQueueGet(message_queue_id, p_message, &msg_prio, timeout);
    if (osOK != status)
    {
        if (osErrorTimeout == status)
        {
            result = OSAL_COM_STATUS_TIMEOUT;
        }
        else if (osErrorResource == status)
        {
            result = OSAL_COM_STATUS_ERROR_RESOURCE;
        }
        else
        {
            SOFTWARE_ASSERT(false);
        }
    }
    return (result);
}
