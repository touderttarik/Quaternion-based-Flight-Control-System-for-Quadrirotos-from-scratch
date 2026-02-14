#include "pid.h"
#define IDENTITY_QUAT quat_make(1.0f, .0f, .0f, .0f) ;



void pid_init(pid_struct_t *pid){
    pid->q_err = quat_make(.0f, .0f, .0f, .0f) ;
    pid->Kr = 1.0f ;
    pid->Kp = 0.2f ; //Muscle
    pid->Ki = 0.0001f ; //The memory
    pid->Kd = 0.0001f ; //The shock absorber. CAUTION : A too high value of Kd will damage the motors.
}

void pid_update(mahony_t *mahony, pid_struct_t *pid, float delta_t, float inverse_delta_t,
    sensor_data_t *sensors, quat_t q_sp){
    quat_normalize(&q_sp);
    // Calculating the error between current attitude and commanded attitude.
    pid->q_err = quat_mul(q_sp, quat_conj(mahony->q_hat));

    if(pid->q_err.w < 0){
        pid->q_err.w *= -1.0f ; 
        pid->q_err.x *= -1.0f ; 
        pid->q_err.y *= -1.0f ; 
        pid->q_err.z *= -1.0f ; 
    }

    //Calculate the vector error
    axis_angle_t axis_angle = quat_to_axis_angle(pid->q_err) ;
    pid->v_err.x = axis_angle.angle*axis_angle.axis.x ;
    pid->v_err.y = axis_angle.angle*axis_angle.axis.y ;
    pid->v_err.z = axis_angle.angle*axis_angle.axis.z ;

    //attitude loop : outputs the angular rate setpoint according to the three axis
    pid->omega_sp.x = pid->Kr*pid->v_err.x ;
    pid->omega_sp.y = pid->Kr*pid->v_err.y ;
    pid->omega_sp.z = pid->Kr*pid->v_err.z ;
    
    //rate loop : CAUTION (CONVERSION TO FRD)
    pid->omega_err.x = pid->omega_sp.x - sensors->gyro.x ; 
    pid->omega_err.y = pid->omega_sp.y + sensors->gyro.y ; //+ because of the FLU to FRD conversion
    pid->omega_err.z = pid->omega_sp.z + sensors->gyro.z ; //+ because of the FLU to FRD conversion

    //Calculating the integral term
    pid->Iomega_err.x = pid->Iomega_err.x + pid->omega_err.x*delta_t;
    pid->Iomega_err.y = pid->Iomega_err.y + pid->omega_err.y*delta_t;
    pid->Iomega_err.z = pid->Iomega_err.z + pid->omega_err.z*delta_t;

    //Calculating the final output
    pid->u.x = pid->Kp*pid->omega_err.x + pid->Ki*pid->Iomega_err.x + pid->Kd*pid->omega_err.x*inverse_delta_t ;
    pid->u.y = pid->Kp*pid->omega_err.y + pid->Ki*pid->Iomega_err.y + pid->Kd*pid->omega_err.y*inverse_delta_t ;
    pid->u.z = pid->Kp*pid->omega_err.z + pid->Ki*pid->Iomega_err.z + pid->Kd*pid->omega_err.z*inverse_delta_t ;

   // printf("[%.02f,%.02f,%.02f,%.02f]\n",pid->q_err.w,pid->q_err.x, pid->q_err.y, pid->q_err.z) ;
    //printf("axis : [%0.2f,%0.2f,%0.2f], angle = %.02f rad\n",axis_angle.axis.x,axis_angle.axis.y,axis_angle.axis.z,axis_angle.angle) ;
}
