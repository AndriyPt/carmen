#ifndef OSAL_IMU_H_
#define OSAL_IMU_H_

#include "osal.h"

void osal_imu_create_thread(osal_thread_function_t *fp_function, uint16_t queue_size);

#endif /* OSAL_IMU_H_ */
