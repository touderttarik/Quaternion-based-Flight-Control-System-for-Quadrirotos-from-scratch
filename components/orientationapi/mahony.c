/**
 * @brief Initialize the Mahony filter structure with default values.
 * 
 * This function initializes a Mahony complementary filter state, which is used
 * for orientation estimation by fusing accelerometer and gyroscope data.
 * 
 * The initialization sets:
 * - The estimated quaternion (q_hat) to identity, representing no initial rotation
 * - The reference vector (e3) pointing downward (gravity direction in body frame)
 * - Proportional gain (kP) to 2.0, balancing responsiveness and stability
 * - Proportional quaternion gain (k1) to 0.0, rarely needed in practice
 * - Integral gain (kI) to 0.0, starting conservative to minimize gyro bias drift
 * - Rotation matrix to identity, representing no initial rotation
 * 
 * Why these defaults?
 * - The identity quaternion ensures the filter starts with a known state
 * - kP of 2.0 provides good responsiveness without excessive noise amplification
 * - kI starts at 0.0 and should be increased gradually only if gyro drift is observed
 * - The identity rotation matrix maintains consistency with the identity quaternion
 * 
 * @param mahony Pointer to the mahony_t structure to initialize.
 *               If NULL, the function returns early without performing initialization.
 * 
 * @return void
 * 
 * @note Always call this function before using the Mahony filter to ensure
 *       all state variables are properly initialized.
 */
#include "mahony.h"
#include "vectors.h"
#define NULL_QUAT quat_make(.0f,.0f,.0f,.0f)
void mahony_init(mahony_t *mahony) {
    
    if (!mahony) {
        return;
    }

    *mahony = (mahony_t){0};
    mahony->q_hat = quat_make(1.0f, 0.0f, 0.0f, 0.0f);
    mahony->q_hat_conj = mahony->q_hat ;
    mahony->q_hat_dot = NULL_QUAT ;
    mahony->e3 = quat_make(0.0f, 0.0f, 0.0f, 1.0f); //Gravity expressed in the world frame as a quaternion
    mahony->q_pure_omega = NULL_QUAT ;
    mahony->k1 = 0.0f;  // Gain proportionnel sur le quaternion (rarement utilisé)
    mahony->kP = 2.0f;  // Gain proportionnel (0.5-5.0 typiquement)
    mahony->kI = 0.2f;  // Gain intégral (commencer à 0, augmenter si bias drift)
    mahony->rot_mat = (matrix_t){{{1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}}};
                                     
}

//Note : The frame Convention of my work is NED for the inertial frame and FRD for the body frame
//       The MPU6050 frame is FLU convention, which means that the proper acceleration vector 
//       is pointing up, which requires a conversion to the FRD convention
//       Adopting NED and FRD convention simplifies greaty the visualization and conversion from the
//       body frame to the world frame. Our quaternion does the body -> world convention
//       Maybe there is a way of configuring the MPU6050 to the FRD convention but I didn't do my research yet

void mahony_update(mahony_t *mahony, sensor_data_t *sensor, const float deltat){
    //va is the measured accelerometer value vector
    mahony -> va.x = sensor -> accel.x ;
    mahony -> va.y = -1*sensor -> accel.y ;// -1* because the sensors frame is in the FLU convention, and our body frame convention is FRD so a conversion is necessary to remain coherent between our different frames
    mahony -> va.z = -1*sensor -> accel.z ;// idem
    vect_normalize(&mahony->va);

    //predict the gravity vector in the body frame by a sandwich product (world to body conversion of the gravity vector e3)
    mahony-> q_hat_conj = quat_conj(mahony->q_hat) ;
    mahony->va_hat_quat = quat_mul(quat_mul(mahony->q_hat_conj,mahony->e3),mahony->q_hat) ;
 
    //We predict the direction of the gravity in the body frame, and the predicted proper acceleration 
    //should point exactly in the opposite direction of the predicted gravity
    //To deduce the predicted proper acceleration, we negate the predicted gravity vector
    //Therefore Mahony compares this predicted proper acceleration with the measured proper acceleration (with the MPU6050) both pointing in the same direction
    //extract vector part of the va_hat quaternion
    mahony->va_hat.x = -1*mahony->va_hat_quat.x ;
    mahony->va_hat.y = -1*mahony->va_hat_quat.y ;
    mahony->va_hat.z = -1*mahony->va_hat_quat.z ;

    //calculate the measurement error by a cross product between va and va_hat
    mahony->omega_mes = cross_prod(mahony->va, mahony->va_hat) ;

    //Updating the bias estimation
    mahony->b_hat_dot.x = mahony->kI*mahony->omega_mes.x ;
    mahony->b_hat_dot.y = mahony->kI*mahony->omega_mes.y ;
    mahony->b_hat_dot.z = mahony->kI*mahony->omega_mes.z ;
    mahony->b_hat.x -= mahony->b_hat_dot.x*deltat;
    mahony->b_hat.y -= mahony->b_hat_dot.y*deltat;
    mahony->b_hat.z -= mahony->b_hat_dot.z*deltat;

    // Correct gyroscope reading with:
        // - Integral feedback (kI) to reduce bias
        // - Proportional feedback (kP) for fast error correction

    mahony->omega.x = sensor->gyro.x - mahony->b_hat.x + mahony->kP*mahony->omega_mes.x ;
    mahony->omega.y = -1*sensor->gyro.y - mahony->b_hat.y + mahony->kP*mahony->omega_mes.y ;
    mahony->omega.z = -1*sensor->gyro.z - mahony->b_hat.z + mahony->kP*mahony->omega_mes.z ;
    
    //convert angular velocity to pure quaternion (scalar part w=0)
    mahony->q_pure_omega.w = 0 ;
    mahony->q_pure_omega.x = mahony->omega.x ;
    mahony->q_pure_omega.y = mahony->omega.y ;
    mahony->q_pure_omega.z = mahony->omega.z ;

    // Quaternion kinematics: q_hat_dot = 0.5 * q_hat * omega
    // This implements the differential equation for quaternion evolution
    mahony->q_hat_dot = quat_mul(mahony->q_hat,mahony->q_pure_omega);
    mahony->q_hat_dot.w *= 0.5f;
    mahony->q_hat_dot.x *= 0.5f;
    mahony->q_hat_dot.y *= 0.5f;
    mahony->q_hat_dot.z *= 0.5f;

    // Euler integration: q_hat(k+1) = q_hat(k) + q_dot(k) * delta_t
    mahony->q_hat.w += mahony->q_hat_dot.w*deltat;
    mahony->q_hat.x += mahony->q_hat_dot.x*deltat;
    mahony->q_hat.y += mahony->q_hat_dot.y*deltat;
    mahony->q_hat.z += mahony->q_hat_dot.z*deltat;
    // Normalize quaternion to maintain unit length constraint
    // Prevents numerical drift and maintains valid rotation representation
    quat_normalize(&mahony->q_hat) ;
}