#ifndef OSAL_CONTROL_LOOP_H_
#define OSAL_CONTROL_LOOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*osal_control_loop_function_t)(void);

void osal_control_loop_create_thread(osal_control_loop_function_t fp_thread_function, uint16_t queue_size,
        uint16_t message_size);

void osal_control_loop_queue_put(const void *p_message);

bool osal_control_loop_queue_get(void *p_message, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* OSAL_CONTROL_LOOP_H_ */
