/**
 * @brief Initialize the Mahony filter structure with default values.
 * 
 * This function initializes a Mahony complementary filter state, which is used
 * for orientation estimation by fusing accelerometer and gyroscope data.
 * 
 * The initialization sets:
 * - The estimated quaternion (q_hat) to identity, representing no initial rotation
 * - The reference vector (e3) pointing downward (gravity direction in body frame)
 * - Proportional gain (kP) to 2.0, balancing responsiveness and stability
 * - Proportional quaternion gain (k1) to 0.0, rarely needed in practice
 * - Integral gain (kI) to 0.0, starting conservative to minimize gyro bias drift
 * - Rotation matrix to identity, representing no initial rotation
 * 
 * Why these defaults?
 * - The identity quaternion ensures the filter starts with a known state
 * - kP of 2.0 provides good responsiveness without excessive noise amplification
 * - kI starts at 0.0 and should be increased gradually only if gyro drift is observed
 * - The identity rotation matrix maintains consistency with the identity quaternion
 * 
 * @param mahony Pointer to the mahony_t structure to initialize.
 *               If NULL, the function returns early without performing initialization.
 * 
 * @return void
 * 
 * @note Always call this function before using the Mahony filter to ensure
 *       all state variables are properly initialized.
 */
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
    mahony->kI = 0.2f;  // Gain intégral (commencer à 0, augmenter si bias drift)
    mahony->rot_mat = (matrix_t){{{1.0f, 0.0f, 0.0f},
                                     {0.0f, 1.0f, 0.0f},
                                     {0.0f, 0.0f, 1.0f}}};
                                     
}
/*void mahony_update(){

}*/
