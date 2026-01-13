#ifndef ORIENTATIONAPI_SENSORS_H
#define ORIENTATIONAPI_SENSORS_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "quaternions.h"

typedef struct {
    vec3_t gyro;  // rad/s
    vec3_t accel; // g
} sensor_data_t;

void mpu6050_init(int scl_gpio, int sda_gpio, uint32_t freq_hz);
void read_sensors(sensor_data_t *out);

#endif /* ORIENTATIONAPI_SENSORS_H */
