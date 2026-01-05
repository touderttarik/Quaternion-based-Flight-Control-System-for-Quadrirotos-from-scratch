#ifndef ORIENTATIONAPI_MAHONY_H
#define ORIENTATIONAPI_MAHONY_H

#include "quaternions.h"
#include "matrix.h"
typedef struct {
    quat_t q_hat;     // peut etre initialise a 1 (repere drone confondu avec repere monde)
    vec3_t omega_y;   // initialise avec sensor.gyro peut etre
    vec3_t omega_mes; // erreur
    float k1;         // magnitude du vecteur normal (lie a l'axe de rotation)
    float kI;
    float kP;
    quat_t e3;        // e3 en quaternion pour predire la gravite. e3 = [0 0 1]
    vec3_t b_hat;     // prediction du biais du gyroscope
    vec3_t va;        // gravite mesuree par l'accelerometre
    vec3_t va_hat;    // prediction de la gravite (ce qu'on devrait mesurer)
    float gyro_error;
    matrix_t rot_mat;   // matrice de rotation associée au quaternion q_hat
} mahony_t;

void mahony_init(mahony_t *mahony);
void mahony_update();


#endif /* ORIENTATIONAPI_MAHONY_H */
