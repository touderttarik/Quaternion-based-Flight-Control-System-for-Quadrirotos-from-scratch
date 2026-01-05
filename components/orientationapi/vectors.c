/**
 * get_vect_norm - Calculate the Euclidean norm of a 3D vector
 * @v: The input vector
 *
 * Return: The magnitude (length) of the vector
 */

/**
 * vect_normalize - Normalize a 3D vector to unit length
 * @v: Pointer to the vector to normalize
 *
 * Modifies the vector in-place to have a magnitude of 1.
 * Does nothing if v is NULL or if the vector norm is zero.
 */

/**
 * cross_prod - Compute the cross product of two 3D vectors
 * @u: The first input vector
 * @v: The second input vector
 *
 * Return: A new vector representing the cross product of u and v
 */
#include "vectors.h"
#include <math.h>

float get_vect_norm(vec3_t v){
    float norm = sqrtf(v.x*v.x+v.y*v.y+v.z*v.z);
    return norm ;
}

void vect_normalize(vec3_t *v){
    float inverse_norm_v;

    if (!v) {
        return;
    }

    inverse_norm_v = 1.0f/get_vect_norm(*v);
    if (inverse_norm_v == 0.0f) {
        return;
    }
    v->x *= inverse_norm_v;
    v->y *= inverse_norm_v;
    v->z *= inverse_norm_v; 
}

vec3_t cross_prod(vec3_t u, vec3_t v) {
    vec3_t res;
    // Les multiplications et soustractions entre float 
    // sont traitées nativement en 1 cycle par la FPU.
    res.x = u.y * v.z - u.z * v.y;
    res.y = u.z * v.x - u.x * v.z;
    res.z = u.x * v.y - u.y * v.x;
    return res;
}

