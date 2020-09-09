#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdint.h>

typedef enum
{
    eeprom_left_motor_p = 0,
    eeprom_left_motor_i,
    eeprom_left_motor_d,

    eeprom_right_motor_p,
    eeprom_right_motor_i,
    eeprom_right_motor_d,
} eeprom_values_t;

void eeprom_get_value(eeprom_values_t name, uint8_t *output, uint16_t length);

void eerprom_create_thread();

#endif /* EEPROM_H_ */
