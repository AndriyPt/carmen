#ifndef OSAL_COMMUNICATION_H_
#define OSAL_COMMUNICATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    OSAL_COM_STATUS_OK,
    OSAL_COM_STATUS_TIMEOUT,
    OSAL_COM_STATUS_ERROR_RESOURCE
} osal_communication_status_t;

#define OSAL_COM_MESSAGE_DATA_SIZE (20)

typedef struct
{
    uint8_t event;
    uint16_t sequence_id;
    uint8_t data[OSAL_COM_MESSAGE_DATA_SIZE];
} osal_communication_message_t;

typedef void (*osal_communication_function_t)(void);

void osal_communication_create_thread(osal_communication_function_t fp_thread_function, uint16_t queue_size);

osal_communication_status_t osal_communication_queue_put(const osal_communication_message_t *p_message);

osal_communication_status_t osal_communication_queue_get(osal_communication_message_t *p_message, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* OSAL_COMMUNICATION_H_ */
