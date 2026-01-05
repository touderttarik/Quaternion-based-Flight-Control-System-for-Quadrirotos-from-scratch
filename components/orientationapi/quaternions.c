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
