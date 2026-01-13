#ifndef ORIENTATIONAPI_PID_H
#define ORIENTATIONAPI_PID_H
#include "mahony.h"
#include "sensors.h"
typedef struct {
    quat_t q_err ; //error quaternion
    vec3_t v_err ; //error vector
    float Kr ; // in s-¹ 
    vec3_t omega_sp ; //angular rate set point
    vec3_t omega_err ; //angular rate error
    vec3_t Iomega_err ; //For the integration of the error
    float Kp ;
    float Ki ;
    float Kd ;
    vec3_t u ;
}pid_struct_t ;


void pid_init(pid_struct_t *pid);
void pid_update(mahony_t *mahony, pid_struct_t *pid, float delta_t, float inverse_delta_t, sensor_data_t *sensors);

#endif