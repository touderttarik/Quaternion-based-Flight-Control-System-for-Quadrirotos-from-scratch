#include "mahony.h"

void mahony_init(mahony_t *mahony) {
    
    if (!mahony) {
        return;
    }

    *mahony = (mahony_t){0};
    mahony->q_hat = quat_make(1.0f, 0.0f, 0.0f, 0.0f);
    mahony->e3 = quat_make(0.0f, 0.0f, 0.0f, 1.0f);
    mahony->k1 = 0.0f;  // Gain proportionnel sur le quaternion (rarement utilisé)
    mahony->kP = 2.0f;  // Gain proportionnel (0.5-5.0 typiquement)
    mahony->kI = 0.0f;  // Gain intégral (commencer à 0, augmenter si bias drift)
    mahony->rot_mat = (matrix_t){{{1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}}};
                                     
}
/*void mahony_update(){

}*/
