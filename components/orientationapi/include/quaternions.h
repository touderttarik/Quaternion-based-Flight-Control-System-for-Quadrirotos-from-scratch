#ifndef ORIENTATIONAPI_QUATERNIONS_H
#define ORIENTATIONAPI_QUATERNIONS_H

typedef struct {float w, x, y, z ;} quat_t ;
typedef struct {float x, y, z; } vec3_t ;

quat_t quat_make(float w, float x, float y, float z) ;
float quat_norm2(quat_t q) ;
quat_t quat_conj(quat_t q) ;
quat_t quat_mul(quat_t a, quat_t b) ;
quat_t quat_normalize (quat_t q) ;
quat_t quat_inverse_unit (quat_t q) ; //Suppose q unitaire
quat_t quat_from_axis_angle(vec3_t axis_unit, float theta_rad) ;
void quat_to_rotmat(quat_t q, float R[3][3]) ;
vec3_t rotmat_apply(const float R[3][3], vec3_t v) ;
vec3_t quat_rotate_vec(quat_t q_unit, vec3_t v) ;

#endif /* ORIENTATIONAPI_QUATERNIONS_H */
