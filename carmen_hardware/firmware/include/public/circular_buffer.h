#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t * p_buffer;
    uint32_t buffer_size;
    uint32_t head_index;
    uint32_t tail_index;
    bool is_full;
} circular_buffer_t;

void circular_buffer_init(circular_buffer_t * p_this, uint8_t * p_buffer, uint32_t size);

void circular_buffer_add(circular_buffer_t * p_this, const uint8_t * p_buffer, uint32_t size);

uint32_t circular_buffer_dequeue(circular_buffer_t * p_this, uint8_t * p_buffer, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_BUFFER_H_ */
