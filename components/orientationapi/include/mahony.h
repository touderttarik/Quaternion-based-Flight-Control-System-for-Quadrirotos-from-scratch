#ifndef ORIENTATIONAPI_MAHONY_H
#define ORIENTATIONAPI_MAHONY_H

#include "quaternions.h"
#include "matrix.h"
#include "sensors.h"
typedef struct {
    quat_t q_hat;     // initialisé a 1 (repere drone confondu avec repere monde)
    quat_t q_hat_conj;//Initialisé à q_hat car initialement q_hat=q_hat*=[1,0,0,0]
    quat_t q_hat_dot; //Used to implemet the differential equation for the evolution of the orientation quaternion over time
    vec3_t omega;   // initialise avec sensor.gyro peut etre
    vec3_t omega_mes; // erreur
    quat_t q_pure_omega ;
    float k1;         // magnitude du vecteur normal (lie a l'axe de rotation)
    float kI;
    float kP;
    quat_t e3;        // e3 en quaternion pour predire la gravite. e3 = [0 0 1]
    vec3_t b_hat;     // prediction du biais du gyroscope
    vec3_t b_hat_dot ;
    vec3_t va;        // gravite mesuree par l'accelerometre
    vec3_t va_hat;    // prediction de la gravite (ce qu'on devrait mesurer)
    quat_t va_hat_quat ; // quaternion dont la partie vectorielle = va_hat
    float gyro_error;
    matrix_t rot_mat;   // matrice de rotation associée au quaternion q_hat
} mahony_t;

void mahony_init(mahony_t *mahony);
void mahony_update(mahony_t *mahony, sensor_data_t *sensor, const float deltat);

#endif