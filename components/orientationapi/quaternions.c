/**
 * @brief Converts a quaternion to Euler angles (pitch, roll, yaw).
 * 
 * Transforms a quaternion representation of rotation into Euler angles using
 * the standard aerospace convention:
 * - Pitch: rotation around the Y-axis (arcsin(2(wy−zx)))
 * - Roll: rotation around the X-axis (arctan2(2(wx+yz), 1−2(x²+y²)))
 * - Yaw: rotation around the Z-axis (arctan2(2(wz+xy), 1−2(y²+z²)))
 * 
 * @param q Pointer to the input quaternion to convert
 * 
 * @return A 3D vector containing pitch, roll, and yaw angles in radians
 *         where: x = pitch, y = roll, z = yaw
 * 
 * @note This function assumes the input quaternion is already normalized.
 *       Results are in radians and typically in the range [-π, π].
 * @note Gimbal lock may occur when pitch approaches ±π/2 radians.
 */
/**
 * @file quaternions.c
 * @brief Quaternion mathematics library for 3D rotations and orientations.
 * 
 * This module provides fundamental quaternion operations including creation,
 * normalization, conjugation, and multiplication. Quaternions are used to
 * represent 3D rotations efficiently and avoid gimbal lock issues.
 */

/**
 * @brief Creates a quaternion with specified components.
 * 
 * @param w The scalar (real) component of the quaternion
 * @param x The x-component of the vector part
 * @param y The y-component of the vector part
 * @param z The z-component of the vector part
 * 
 * @return A quaternion structure with the specified components
 */

/**
 * @brief Calculates the norm (magnitude) of a quaternion.
 * 
 * Computes the Euclidean norm using the formula: sqrt(w² + x² + y² + z²)
 * 
 * @param q The input quaternion
 * 
 * @return The norm value of the quaternion
 */

/**
 * @brief Computes the conjugate of a quaternion.
 * 
 * The conjugate negates the vector part (x, y, z) while keeping the scalar
 * part (w) unchanged. For unit quaternions, this represents the inverse
 * rotation and can be interpreted as rotating by angle -theta around axis u,
 * or by angle theta around axis -u.
 * 
 * @param q The input quaternion
 * 
 * @return The conjugate quaternion
 * 
 * @note Assumes unit quaternions (already normalized)
 */

/**
 * @brief Multiplies two quaternions using the Hamilton product.
 * 
 * Performs quaternion multiplication which combines two rotations.
 * Note: Quaternion multiplication is non-commutative (a*b ≠ b*a).
 * 
 * @param a The first quaternion operand
 * @param b The second quaternion operand
 * 
 * @return The product quaternion representing the combined rotation
 */

/**
 * @brief Normalizes a quaternion to unit length.
 * 
 * Scales the quaternion components so that its norm equals 1.
 * Uses inverse norm calculation and multiplication instead of division
 * for better performance on ESP32 FPU (single-cycle multiplication vs
 * slow division).
 * 
 * @param q The input quaternion to normalize
 * 
 * @return The normalized unit quaternion
 */
#include "quaternions.h"
#include "math.h"

quat_t quat_make(float w, float x, float y, float z){
    quat_t q ;
    q.w = w ;
    q.x = x ;
    q.y = y ;
    q.z = z ;
    return q ;
}

float quat_norm2(quat_t q) {
    float q_norm ;
    q_norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return q_norm ;
}

quat_t quat_conj(quat_t q){//Utiliser aussi comme trouver l'inverse d'un quaternion, on suppose qu'on travaille avec des quaternions unitaires donc déjà normalisés
/*Le conjugué s'interprète comme une rotation d'angle -theta autour d'un axe u 
ou une rotation d'angle theta autour de l'axe -u*/
    quat_t q_conj   ;
    q_conj.w = q.w;
    q_conj.x = -q.x;
    q_conj.y = -q.y;
    q_conj.z = -q.z;
    return q_conj ;
}

quat_t quat_mul(quat_t a, quat_t b){
    quat_t q_prod ;
    q_prod.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z ;
    q_prod.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y ;
    q_prod.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x ;
    q_prod.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w ;
    return q_prod ;
}

void quat_normalize (quat_t *q){
/*Utilité de la normalisaion :
*/
    //le choix d'utiliser inverse_norm au lieu de norm est dans le but de profiter
    //de la FPU de l'esp32 qui fait des multiplications en un cycle
    //la division étant très lente sur esp32
    float inverse_norm = 1.0f/quat_norm2(*q);
    q->w *= inverse_norm ;
    q->x *= inverse_norm ; 
    q->y *= inverse_norm ;
    q->z *= inverse_norm ;
}

axis_angle_t quat_to_axis_angle(quat_t q){
    axis_angle_t axis_angle ;
    float temp = .0f;
    if(q.w == 1){ //if w=1 there is no rotation (body frame aligned with intertial frame, so the angle of rotation =0 (no rotation) and any axis will do)
        axis_angle.axis.x = 0 ;
        axis_angle.axis.y = 0 ;
        axis_angle.axis.z = 0 ;
        axis_angle.angle = 0 ;
    }
    else{
        if(q.w<1){
            temp = 1/sqrtf(1-q.w*q.w) ;
            axis_angle.angle = 2.0f*acosf(q.w) ;
            axis_angle.axis.x = q.x * temp;
            axis_angle.axis.y = q.y * temp;
            axis_angle.axis.z = q.z * temp;
        }
    }
    return axis_angle;
}

/*vec3_t quat_to_pitch_roll_yaw(quat_t *q) {
    //pitch=arcsin(2(wy−zx))
    //roll=arctan2(2(wx+yz),1−2(x²+y²))
    //yaw=arctan2(2(wz+xy),1−2(y²+z²))
    vec3_t pitch_roll_yaw ;
    pitch_roll_yaw.x = asinf(2*(q->w*q->y - q->z*q->x));//pitch
    pitch_roll_yaw.y = atan2f(2*(q->w*q->x + q->y*q->z),1-2*(q->x*q->x+q->y*q->y));//roll
    pitch_roll_yaw.z = atan2f(2*(q->w*q->z + q->x*q->y),1-2*(q->y*q->y+q->z*q->z));//yaw
}*/
