#ifndef ORIENTATIONAPI_MATRIX_H
#define ORIENTATIONAPI_MATRIX_H

#include "quaternions.h"

typedef struct {
    float R[3][3] ;
} matrix_t ;


matrix_t get_skew_mat(vec3_t v);

vec3_t mat_vec_prod(matrix_t mat, vec3_t v) ;

#endif