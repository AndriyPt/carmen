#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

void send_new_command_event(void);

void communication_init(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H_ */
