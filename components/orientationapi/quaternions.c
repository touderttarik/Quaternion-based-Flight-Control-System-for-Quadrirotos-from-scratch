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

quat_t quat_normalize (quat_t q){
/*Utilité de la normalisaion :
*/
    //le choix d'utiliser inverse_norm au lieu de norm est dans le but de profiter
    //de la FPU de l'esp32 qui fait des multiplications en un cycle
    //la division étant très lente sur esp32
    quat_t normalized_quat ;
    float inverse_norm = 1.0f/quat_norm2(q);
    normalized_quat.w = q.w*inverse_norm ;
    normalized_quat.x = q.x*inverse_norm ; 
    normalized_quat.y = q.y*inverse_norm ;
    normalized_quat.z = q.z*inverse_norm ;
    return normalized_quat ;
}
