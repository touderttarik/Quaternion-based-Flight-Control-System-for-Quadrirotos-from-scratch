#include "vectors.h"
#include <math.h>

float get_vect_norm(vec3_t v){
    float norm = sqrt(pow(v.x,2)+pow(v.y,2)+pow(v.z,2));
    return norm ;
}

void vect_normalize(vec3_t *v){
    float norm_v;

    if (!v) {
        return;
    }

    norm_v = get_vect_norm(*v);
    if (norm_v == 0.0f) {
        return;
    }
    v->x /= norm_v;
    v->y /= norm_v;
    v->z /= norm_v;
}

