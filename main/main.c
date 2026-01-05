#include <math.h>
#include <stdio.h>
#include "quaternions.h"
#include "vectors.h"
#include "matrix.h"
#include "mahony.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"

#define MPU6050_ADD 0x68
#define WAKEUP_VAL 0x00
#define REGISTERS_ADD 0x3B
#define NULL_QUAT quat_make(0.0f, 0.0f, 0.0f, 0.0f)
// Goal: estimate rotation matrix R and bias b to correct gyroscope drift

void app_main(void)
{
    // Decimation factor to reduce output frequency from 500Hz to 50Hz
    const int plot_decimation = 10;
    int plot_counter = 0;
    // Conversion constants for angle transformations
    const float rad_to_deg = 57.2957795f;
    const float deg_to_rad = M_PI/180.f;
    
    // Configure I2C master bus for communication with MPU6050
    i2c_master_bus_config_t i2cBus = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 16,
        .sda_io_num = 17,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    
    i2c_master_bus_handle_t bus_handle;
    
    // Initialize the I2C bus
    i2c_new_master_bus(&i2cBus, &bus_handle);

    // Configure I2C device (MPU6050 slave)
    i2c_device_config_t i2cdev = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU6050_ADD,
        .scl_speed_hz = 400000,
        .scl_wait_us = 10
    };

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &i2cdev, &dev_handle);
    
    // Data structure to hold accelerometer and gyroscope readings
    typedef struct {
        vec3_t gyro;    // Angular velocity from gyroscope (rad/s)
        vec3_t accel;   // Linear acceleration from accelerometer (g)
    } sensor_data_t;
    
    // Wake up the MPU6050 sensor (exit sleep mode)
    uint8_t wakeupbuff[2] = {MPU6050_ADD, WAKEUP_VAL};
    i2c_master_transmit(dev_handle, wakeupbuff, 2, 1000);
    
    // Output headers for plotting different data streams
    printf("PLOT_G_HEADER,va_x[u],va_y[u],va_z[u],va_hat_x[u],va_hat_y[u],va_hat_z[u]\n");
    printf("PLOT_BIAS_HEADER,bias_hat_x[arb],bias_hat_y[arb],bias_hat_z[arb],i_corr_x[rad/s],i_corr_y[rad/s],i_corr_z[rad/s]\n");
    printf("PLOT_Q_HEADER,q_w[u],q_x[u],q_y[u],q_z[u],q_norm[u]\n");
    printf("PLOT_SENS_HEADER,accel_x[g],accel_y[g],accel_z[g],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s]\n");
    printf("PLOT_ERR_HEADER,err_angle_deg[deg],omega_mes_x[arb],omega_mes_y[arb],omega_mes_z[arb]\n");

    // Initialize Mahony filter state
    mahony_t mahony;
    mahony_init(&mahony);
    
    // Sampling period: 2ms for 500Hz loop frequency
    float delta_t = 0.002f;
    sensor_data_t sensors = {0};
    
    // Quaternion derivative (rate of change of orientation)
    quat_t q_hat_dot = NULL_QUAT;
    // Conjugate of estimated quaternion (for rotation operations)
    quat_t q_hat_conj = NULL_QUAT;
    // Temporary quaternion for sandwich product calculations
    quat_t quat_temp = NULL_QUAT;
    
    // Predicted gravity vector in body frame
    vec3_t va_hat = {0};
    // Quaternion representation of predicted gravity
    quat_t va_hat_quat = NULL_QUAT;
    
    // Cross product error between measured and predicted gravity
    // Used to correct gyroscope bias and attitude
    vec3_t omega_mes = {0};
    

    // Corrected angular velocity (gyro reading + bias correction + proportional gain correction)
    vec3_t omega = {0};
    
    // Pure quaternion representation of angular velocity (scalar part = 0)
    quat_t q_pure_omega = NULL_QUAT;
    
    // Estimated gyroscope bias (accumulates over time)
    vec3_t bias_hat = {0};
    vec3_t bias_hat_dot = {0};
    
    // Rotation matrix (for future use in filter improvements)
    float R[3][3] = {0};
    
    // I2C register address for reading sensor data
    uint8_t DataReg = REGISTERS_ADD ;
    // Buffer to store 14 bytes of sensor data (6 accel + 2 temp + 6 gyro)
    uint8_t SensorData[14] ;
    
    // Scaling factors to convert raw sensor values to physical units
    const float raw_accel_const = 1 / 16384.0f;  // ±8g range
    const float raw_gyro_const = deg_to_rad / 131.0f;  // ±1000°/s range
    
    // Main control loop
    for(;;) {
        // Read 14 bytes from sensor registers via I2C
        i2c_master_transmit_receive(dev_handle, &DataReg, 1, SensorData, 14, 1000);
        
        // Extract and scale accelerometer data (3-axis, 2 bytes per axis)
        sensors.accel.x = ((int16_t)((SensorData[0] << 8) | SensorData[1])) * raw_accel_const;
        sensors.accel.y = ((int16_t)((SensorData[2] << 8) | SensorData[3])) * raw_accel_const;
        sensors.accel.z = ((int16_t)((SensorData[4] << 8) | SensorData[5])) * raw_accel_const;
        
        // Extract and scale gyroscope data (skip temperature bytes at indices 6-7)
        sensors.gyro.x = ((int16_t)((SensorData[8] << 8) | SensorData[9])) * raw_gyro_const;
        sensors.gyro.y = ((int16_t)((SensorData[10] << 8) | SensorData[11])) * raw_gyro_const;
        sensors.gyro.z = ((int16_t)((SensorData[12] << 8) | SensorData[13])) * raw_gyro_const;
        
        // Normalize measured acceleration (gravity vector) to unit length
        // This gives us the direction of gravity regardless of magnitude variations
        mahony.va.x = sensors.accel.x;
        mahony.va.y = sensors.accel.y;
        mahony.va.z = sensors.accel.z;
        vect_normalize(&mahony.va);
        
        // Predict expected gravity direction in body frame using estimated quaternion
        // Apply: va_hat = q_hat^-1 * e3 * q_hat (sandwich product)
        // e3 is the gravity vector in world frame [0, 0, 1, 0]
        q_hat_conj = quat_conj(mahony.q_hat) ;
        quat_temp = quat_mul(q_hat_conj, mahony.e3) ;
        va_hat_quat = quat_mul(quat_temp, mahony.q_hat) ;
        
        // Extract vector part of predicted gravity quaternion
        //Il faut qu'il y a it un va_hat_quat.w proche de 0
        va_hat.x = va_hat_quat.x;
        va_hat.y = va_hat_quat.y;
        va_hat.z = va_hat_quat.z;
        
        // Calculate measurement error: cross product gives normal vector to rotation plane
        // This error drives the attitude correction loop
        omega_mes = cross_prod(mahony.va, va_hat) ;
        bias_hat_dot.x = mahony.kI*omega_mes.x ;
        bias_hat_dot.y = mahony.kI*omega_mes.y ;
        bias_hat_dot.z = mahony.kI*omega_mes.z ;

        // Integral term accumulates the error to correct gyroscope bias
        // Prevents drift by continuously adjusting the bias estimate downward
        bias_hat.x = bias_hat.x - bias_hat_dot.x * delta_t;
        bias_hat.y = bias_hat.y - bias_hat_dot.y * delta_t;
        bias_hat.z = bias_hat.z - bias_hat_dot.z * delta_t;
        
        // Correct gyroscope reading with:
        // - Integral feedback (kI) to reduce bias
        // - Proportional feedback (kP) for fast error correction
        omega.x = sensors.gyro.x - bias_hat.x + mahony.kP * omega_mes.x;
        omega.y = sensors.gyro.y - bias_hat.y + mahony.kP * omega_mes.y;
        omega.z = sensors.gyro.z - bias_hat.z + mahony.kP * omega_mes.z;
        
        // Convert angular velocity to pure quaternion (scalar part = 0)
        q_pure_omega.w = 0;
        q_pure_omega.x = omega.x;
        q_pure_omega.y = omega.y;
        q_pure_omega.z = omega.z;
        
        // Quaternion kinematics: q_dot = 0.5 * q * omega
        // This implements the differential equation for quaternion evolution
        q_hat_dot = quat_mul(mahony.q_hat, q_pure_omega);
        q_hat_dot.w = q_hat_dot.w * 0.5f ;
        q_hat_dot.x = q_hat_dot.x * 0.5f ;
        q_hat_dot.y = q_hat_dot.y * 0.5f ;
        q_hat_dot.z = q_hat_dot.z * 0.5f ;
        
        // Euler integration: q_hat(k+1) = q_hat(k) + q_dot(k) * delta_t
        mahony.q_hat.w = mahony.q_hat.w + q_hat_dot.w * delta_t;
        mahony.q_hat.x = mahony.q_hat.x + q_hat_dot.x * delta_t;
        mahony.q_hat.y = mahony.q_hat.y + q_hat_dot.y * delta_t;
        mahony.q_hat.z = mahony.q_hat.z + q_hat_dot.z * delta_t;
        
        // Normalize quaternion to maintain unit length constraint
        // Prevents numerical drift and maintains valid rotation representation
        quat_normalize(&mahony.q_hat);



        // Conditional plotting to reduce serial communication overhead
        if ((plot_counter++ % plot_decimation) == 0) {
            // Calculate quaternion norm as sanity check
            float q_norm = sqrtf(
                mahony.q_hat.w * mahony.q_hat.w +
                mahony.q_hat.x * mahony.q_hat.x +
                mahony.q_hat.y * mahony.q_hat.y +
                mahony.q_hat.z * mahony.q_hat.z);
            
            // Calculate dot product between measured and predicted gravity
            // Clamping prevents numerical issues with acos
            float va_dot = mahony.va.x * va_hat.x +
                           mahony.va.y * va_hat.y +
                           mahony.va.z * va_hat.z;
            if (va_dot > 1.0f) va_dot = 1.0f;
            if (va_dot < -1.0f) va_dot = -1.0f;
            
            // Convert error to angle in degrees
            float err_angle_deg = acosf(va_dot) * rad_to_deg;
            
            // Calculate integral correction terms
            float i_corr_x = mahony.kI * bias_hat.x;
            float i_corr_y = mahony.kI * bias_hat.y;
            float i_corr_z = mahony.kI * bias_hat.z;

            // Output measured vs predicted gravity vectors
            printf("PLOT_G,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   mahony.va.x, mahony.va.y, mahony.va.z,
                   va_hat.x, va_hat.y, va_hat.z);
            
            // Output bias estimate and integral correction
            printf("PLOT_BIAS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   bias_hat.x, bias_hat.y, bias_hat.z,
                   i_corr_x, i_corr_y, i_corr_z);
            
            // Output estimated quaternion and its norm
            printf("PLOT_Q,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   mahony.q_hat.w, mahony.q_hat.x, mahony.q_hat.y,
                   mahony.q_hat.z, q_norm);
            
            // Output raw sensor readings
            printf("PLOT_SENS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   sensors.accel.x, sensors.accel.y, sensors.accel.z,
                   sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
            
            // Output attitude error and measurement error vector
            printf("PLOT_ERR,%.6f,%.6f,%.6f,%.6f\n",
                   err_angle_deg, omega_mes.x, omega_mes.y, omega_mes.z);
        }
        
        // Delay 2ms to maintain 500Hz control loop frequency
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
