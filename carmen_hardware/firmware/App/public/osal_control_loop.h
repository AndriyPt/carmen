#ifndef OSAL_CONTROL_LOOP_H_
#define OSAL_CONTROL_LOOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    OSAL_CL_STATUS_OK,
    OSAL_CL_STATUS_TIMEOUT,
    OSAL_CL_STATUS_ERROR_RESOURCE
} osal_control_loop_status_t;

#define OSAL_CL_MESSAGE_DATA_SIZE (20)

typedef struct
{
    uint8_t message_type;
    uint8_t data[OSAL_CL_MESSAGE_DATA_SIZE];
} osal_cl_message_t;

typedef void (*osal_control_loop_function_t)(void);

void osal_control_loop_create_thread(osal_control_loop_function_t fp_thread_function, uint16_t queue_size);

osal_control_loop_status_t osal_control_loop_queue_put(const osal_cl_message_t *p_message);

osal_control_loop_status_t osal_control_loop_queue_get(osal_cl_message_t *p_message, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* OSAL_CONTROL_LOOP_H_ */
