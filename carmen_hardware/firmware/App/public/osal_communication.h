#ifndef OSAL_COMMUNICATION_H_
#define OSAL_COMMUNICATION_H_

typedef void (osal_communication_function_t)(void);

void osal_communication_create_thread(osal_communication_function_t *thread_function);

#endif /* OSAL_COMMUNICATION_H_ */
