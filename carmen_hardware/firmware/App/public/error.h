#ifndef ERROR_H_
#define ERROR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <assert.h>

typedef enum
{
    ERROR_SOFTWARE,
    ERROR_HARDWARE,
    ERROR_CRC_CHECK
} error_t;

// TODO: Implement error handling

#define SOFTWARE_ERROR(X) assert(!(X))

#define HARDWARE_ERROR(X) assert(!(X))

#define CHECK_ERROR(X, Y) assert(!(X))

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H_ */
