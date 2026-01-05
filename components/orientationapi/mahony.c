#include "mahony.h"

void mahony_init(mahony_t *mahony) {
    
    if (!mahony) {
        return;
    }

    *mahony = (mahony_t){0};
    mahony->q_hat = quat_make(1.0f, 0.0f, 0.0f, 0.0f);
    mahony->e3 = quat_make(0.0f, 0.0f, 0.0f, 1.0f);
    mahony->k1 = 1.0f ;
    mahony->kP = 5.0f ;
    mahony->kI = 0.05f ; 
    mahony->rot_mat = (matrix_t){{{1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}}};
                                     
}
/*void mahony_update(){

}*/
