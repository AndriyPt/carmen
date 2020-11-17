#ifndef ERROR_H_
#define ERROR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef enum
{
    ERROR_SOFTWARE = 1,
    ERROR_HARDWARE,
    ERROR_CRC_CHECK
} error_types_t;

void generate_error(error_types_t error_type, uint8_t* p_file_name, uint32_t line_number);

#define SOFTWARE_ASSERT(X) ((X) ? (void)0U : generate_error(ERROR_SOFTWARE, (uint8_t*)__FILE__, __LINE__))

#define HARDWARE_ASSERT(X) ((X) ? (void)0U : generate_error(ERROR_HARDWARE, (uint8_t*)__FILE__, __LINE__))

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H_ */
