#include "pid.h"
#include "mix.h"

float clamp(float n, float min, float max){
    if(n<min) return min ;
    if(n>max) return max ;
    return n ;
}


void mix_update(pid_struct_t *pid, mix_t *mix, float throttle){

    mix->f1 = clamp(throttle + pid->u.y + pid->u.x - pid->u.z, 0, 1.0 );
    mix->f2 = clamp(throttle + pid->u.y - pid->u.x + pid->u.z, 0, 1.0 );
    mix->f3 = clamp(throttle - pid->u.y - pid->u.x - pid->u.z, 0, 1.0 );
    mix->f4 = clamp(throttle - pid->u.y + pid->u.x + pid->u.z, 0, 1.0 );
    
}

