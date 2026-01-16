#ifndef ORIENTATIONAPI_MIX_H
#define ORIENTATIONAPI_MIX_H
#include "pid.h"
#include "stdint.h"
typedef struct {
    float f1; 
    float f2; 
    float f3; 
    float f4;
    uint16_t pwm_1;
    uint16_t pwm_2;
    uint16_t pwm_3;
    uint16_t pwm_4;
} mix_t;

float clamp(float n, float min, float max) ;
void mix_update(pid_struct_t *pid, mix_t *mix) ;

#endif
