#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*control_loop_result_t)(uint16_t sequence_id, bool result);

typedef struct
{
    uint16_t sequence_id;
    uint16_t left_cmd;
    uint16_t right_cmd;
    control_loop_result_t result_callback;
} control_loop_set_command_t;

typedef struct
{
    uint16_t sequence_id;
    uint16_t p;
    uint16_t i;
    uint16_t d;
    control_loop_result_t result_callback;
} control_loop_set_pid_t;

void control_loop_init();

void control_loop_set_commands(const control_loop_set_command_t *message);

void control_loop_set_pid(const control_loop_set_pid_t *message);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H_ */
