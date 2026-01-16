#ifndef ORIENTATION_API_MOTORS_H
#define ORIENTATION_API_MOTORS_H
#include "driver/mcpwm_prelude.h"
#include "mix.h"


void motors_init(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin);
void write_motors(mix_t *mix) ;


#endif
