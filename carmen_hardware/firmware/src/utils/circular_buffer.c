#include "circular_buffer.h"
#include "error.h"
#include <string.h>
#include <stdint.h>

void circular_buffer_init(circular_buffer_t * p_this, uint8_t * p_buffer, uint32_t size)
{
    SOFTWARE_ASSERT(NULL != p_this);
    SOFTWARE_ASSERT(NULL != p_buffer);
    SOFTWARE_ASSERT(0 < size);

    p_this->p_buffer = p_buffer;
    p_this->buffer_size = size;
    p_this->head_index = 0;
    p_this->tail_index = 0;
    p_this->is_full = false;
}

void circular_buffer_add(circular_buffer_t * p_this, const uint8_t * p_buffer, uint32_t size)
{
    SOFTWARE_ASSERT(NULL != p_this);
    SOFTWARE_ASSERT(NULL != p_this->p_buffer);
    SOFTWARE_ASSERT(0 != p_this->buffer_size);
    SOFTWARE_ASSERT(NULL != p_buffer);
    SOFTWARE_ASSERT(0 != size);
    SOFTWARE_ASSERT(false == p_this->is_full);

    if (p_this->tail_index >= p_this->head_index)
    {
        SOFTWARE_ASSERT((p_this->buffer_size - (p_this->tail_index - p_this->head_index)) >= size);
        if ((p_this->tail_index + size) < p_this->buffer_size)
        {
            memcpy(&p_this->p_buffer[p_this->tail_index], &p_buffer[0], size);
            p_this->tail_index += size;
        }
        else
        {
            uint32_t remaning_size = p_this->buffer_size - p_this->tail_index;
            memcpy(&p_this->p_buffer[p_this->tail_index], &p_buffer[0], remaning_size);
            memcpy(&p_this->p_buffer[0], &p_buffer[remaning_size], size - remaning_size);
            p_this->tail_index = size - remaning_size;
        }
    }
    else
    {
        SOFTWARE_ASSERT((p_this->head_index - p_this->tail_index) >= size);
        memcpy(&p_this->p_buffer[p_this->tail_index], &p_buffer[0], size);
        p_this->tail_index += size;
    }
    if (p_this->tail_index == p_this->head_index)
    {
        p_this->is_full = true;
    }
}

uint32_t circular_buffer_dequeue(circular_buffer_t * p_this, uint8_t * p_buffer, uint32_t size)
{
    SOFTWARE_ASSERT(NULL != p_this);
    SOFTWARE_ASSERT(NULL != p_this->p_buffer);
    SOFTWARE_ASSERT(0 != p_this->buffer_size);
    SOFTWARE_ASSERT(NULL != p_buffer);
    SOFTWARE_ASSERT(0 != size);

    uint32_t result = 0;

    if ((p_this->head_index == p_this->tail_index) && !p_this->is_full)
    {
        return result;
    }
    if (p_this->head_index < p_this->tail_index)
    {
        result = p_this->tail_index - p_this->head_index;
        if (result > size)
        {
            result = size;
        }
        memcpy(&p_buffer[0], &p_this->p_buffer[p_this->head_index], result);
        p_this->head_index += result;
    }
    else
    {
        if ((p_this->buffer_size - p_this->head_index) > size)
        {
            result = size;
            memcpy(&p_buffer[0], &p_this->p_buffer[p_this->head_index], result);
            p_this->head_index += result;
        }
        else
        {
            result = p_this->buffer_size - p_this->head_index;
            memcpy(&p_buffer[0], &p_this->p_buffer[p_this->head_index], result);

            if (p_this->tail_index > (size - result))
            {
                memcpy(&p_buffer[result], &p_this->p_buffer[0], size - result);
                p_this->head_index = size - result;
                result = size;
            }
            else
            {
                memcpy(&p_buffer[result], &p_this->p_buffer[0], p_this->tail_index);
                p_this->head_index = p_this->tail_index;
                result += p_this->tail_index;
            }
        }
        p_this->is_full = false;
    }
    return result;
}
