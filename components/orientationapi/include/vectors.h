#ifndef ORIENTATIONAPI_VECTORS_H
#define ORIENTATIONAPI_VECTORS_H

#include "quaternions.h"

float get_vect_norm(vec3_t v);
void vect_normalize(vec3_t *v);
vec3_t cross_prod(vec3_t v1, vec3_t v2) ;



#endif /* ORIENTATIONAPI_VECTORS_H */
