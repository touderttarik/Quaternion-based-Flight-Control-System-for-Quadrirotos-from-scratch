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
//But du filtre : estimer R et estimer b pour corriger le gyro

void app_main(void)

{
    const int plot_decimation = 10; // reduce serial spam: 500 Hz / 10 = 50 Hz
    int plot_counter = 0;
    const float rad_to_deg = 57.2957795f;
    const float deg_to_rad = M_PI/180.f;
    //Configuration du bus maitre et du périphérique esclave (mpu6050)
    i2c_master_bus_config_t i2cBus = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 16,
        .sda_io_num = 17,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    
    i2c_master_bus_handle_t bus_handle;
    
    i2c_new_master_bus(&i2cBus,&bus_handle);

    i2c_device_config_t i2cdev = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU6050_ADD,
        .scl_speed_hz = 400000,
        .scl_wait_us = 10
    };

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle,&i2cdev,&dev_handle);
    typedef struct { //placer dans une bibliothèque sensor.h qui renvoie les données en rad/s
        vec3_t gyro ; //gyro[0] = axe x etc...
        vec3_t accel;
    } sensor_data_t ;
    //Réveiller le capteur
    uint8_t wakeupbuff[2] = {MPU6050_ADD, WAKEUP_VAL};
    i2c_master_transmit(dev_handle, wakeupbuff, 2, 1000);
    printf("PLOT_G_HEADER,va_x[u],va_y[u],va_z[u],va_hat_x[u],va_hat_y[u],va_hat_z[u]\n");
    printf("PLOT_BIAS_HEADER,bias_hat_x[arb],bias_hat_y[arb],bias_hat_z[arb],i_corr_x[rad/s],i_corr_y[rad/s],i_corr_z[rad/s]\n");
    printf("PLOT_Q_HEADER,q_w[u],q_x[u],q_y[u],q_z[u],q_norm[u]\n");
    printf("PLOT_SENS_HEADER,accel_x[g],accel_y[g],accel_z[g],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s]\n");
    printf("PLOT_ERR_HEADER,err_angle_deg[deg],omega_mes_x[arb],omega_mes_y[arb],omega_mes_z[arb]\n");


    //variables mahony
    mahony_t mahony ;
    mahony_init(&mahony) ;
    float delta_t = 0.002f ; //mesure toutes les 0.002 seconde (boucle à 500 Hz)
    sensor_data_t sensors= {0};
    quat_t q_hat_dot = quat_make (0, 0, 0, 0) ;
    quat_t va_quat = quat_make(0, mahony.va.x, mahony.va.y, mahony.va.z);
    quat_t q_hat_conj = quat_make(0,0,0,0);//init à n'importe quoi
    quat_t quat_temp = quat_make(0,0,0,0) ;// sera utilisé pour la partie gauche du sandwich
    vec3_t va_hat = {0} ;
    quat_t va_hat_quat = quat_make(0,0,0,0) ;// exprimé le vecteur prédiction sous forme de quaternion
    vec3_t omega_mes ; //erreur prédiction-mesure capeur (un vecteur normal au plan de va_hat et va)
    vec3_t omega ; //vitesse du gyro corrigée (inclut la lecture du gyro Omega_y, le bias prédit et l'erreur)
    quat_t q_pure_omega = quat_make(0, 0, 0, 0) ;//expression de omega sous forme de quaternion pur
    vec3_t bias_hat = {0} ;
    float R[3][3] = {0} ; //
    //variables de lecture de données
    uint8_t DataReg = REGISTERS_ADD ;
    uint8_t AccelReg = 0x3B ;
    uint8_t GyroReg = 0x43 ;
    int16_t xAccelraw = 0 ;
    int16_t yAccelraw = 0 ;
    int16_t zAccelraw = 0 ;
    int16_t xGyroraw = 0 ;
    int16_t yGyroraw = 0 ;
    int16_t zGyroraw = 0 ;
    uint8_t SensorData[14] ;
    const float raw_accel_const = 1 / 16384.0f;
    const float raw_gyro_const = deg_to_rad / 131.0f;
    for(;;){
        i2c_master_transmit_receive_async(dev_handle, &DataReg, 1, SensorData, 14, NULL);
        
        sensors.accel.x = ((int16_t)((SensorData[0] << 8) | SensorData[1])) * raw_accel_const;
        sensors.accel.y = ((int16_t)((SensorData[2] << 8) | SensorData[3])) * raw_accel_const;
        sensors.accel.z = ((int16_t)((SensorData[4] << 8) | SensorData[5])) * raw_accel_const;
        
        sensors.gyro.x = ((int16_t)((SensorData[8] << 8) | SensorData[9])) * raw_gyro_const;
        sensors.gyro.y = ((int16_t)((SensorData[10] << 8) | SensorData[11])) * raw_gyro_const;
        sensors.gyro.z = ((int16_t)((SensorData[12] << 8) | SensorData[13])) * raw_gyro_const;
        
        //normaliser va
        mahony.va.x = sensors.accel.x ;
        mahony.va.y = sensors.accel.y ;
        mahony.va.z = sensors.accel.z ;
        vect_normalize(&mahony.va) ;
        
        //prédire la gravité va_hat
        q_hat_conj = quat_conj(mahony.q_hat) ;
        quat_temp = quat_mul(q_hat_conj, mahony.e3) ;
        va_hat_quat = quat_mul(quat_temp,mahony.q_hat);
        //printf("va x = %f | va y = %f | va_z = %f \n",mahony.va.x, mahony.va.y, mahony.va.z);
        //calculer l'erreur omeag mes
        va_hat.x = va_hat_quat.x ;
        va_hat.y = va_hat_quat.y ;
        va_hat.z = va_hat_quat.z ;
        omega_mes = cross_prod(mahony.va, va_hat) ;
        bias_hat.x = bias_hat.x - omega_mes.x ;
        bias_hat.y = bias_hat.y - omega_mes.y ;
        bias_hat.z = bias_hat.z - omega_mes.z ;
        omega.x =  sensors.gyro.x - mahony.kI*bias_hat.x + mahony.kP*omega_mes.x ;
        omega.y =  sensors.gyro.y - mahony.kI*bias_hat.y + mahony.kP*omega_mes.y ;
        omega.z =  sensors.gyro.z - mahony.kI*bias_hat.z + mahony.kP*omega_mes.z ;
        //cinématique quaternion
        q_pure_omega.w = 0 ;
        q_pure_omega.x = omega.x  ;
        q_pure_omega.y = omega.y  ;
        q_pure_omega.z = omega.z ;
        //mise à jour du quaternion orientation 
        q_hat_dot = quat_mul(mahony.q_hat,q_pure_omega) ;
        q_hat_dot.x = q_hat_dot.x * 0.5f ;
        q_hat_dot.y = q_hat_dot.y * 0.5f ;
        q_hat_dot.z = q_hat_dot.z * 0.5f ;
        //intégrer
        mahony.q_hat.x = mahony.q_hat.x + q_hat_dot.x*delta_t ;
        mahony.q_hat.y = mahony.q_hat.y + q_hat_dot.y*delta_t ;
        mahony.q_hat.z = mahony.q_hat.z + q_hat_dot.z*delta_t ;
        mahony.q_hat   = quat_normalize(mahony.q_hat) ;


        //partie plot

        if ((plot_counter++ % plot_decimation) == 0) {
            float q_norm = sqrtf(
                mahony.q_hat.w * mahony.q_hat.w +
                mahony.q_hat.x * mahony.q_hat.x +
                mahony.q_hat.y * mahony.q_hat.y +
                mahony.q_hat.z * mahony.q_hat.z);
            float va_dot = mahony.va.x * va_hat.x +
                           mahony.va.y * va_hat.y +
                           mahony.va.z * va_hat.z;
            if (va_dot > 1.0f) va_dot = 1.0f;
            if (va_dot < -1.0f) va_dot = -1.0f;
            float err_angle_deg = acosf(va_dot) * rad_to_deg;
            float i_corr_x = mahony.kI * bias_hat.x;
            float i_corr_y = mahony.kI * bias_hat.y;
            float i_corr_z = mahony.kI * bias_hat.z;

            printf("PLOT_G,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   mahony.va.x, mahony.va.y, mahony.va.z,
                   va_hat.x, va_hat.y, va_hat.z);
            printf("PLOT_BIAS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   bias_hat.x, bias_hat.y, bias_hat.z,
                   i_corr_x, i_corr_y, i_corr_z);
            printf("PLOT_Q,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   mahony.q_hat.w, mahony.q_hat.x, mahony.q_hat.y,
                   mahony.q_hat.z, q_norm);
            printf("PLOT_SENS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   sensors.accel.x, sensors.accel.y, sensors.accel.z,
                   sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
            printf("PLOT_ERR,%.6f,%.6f,%.6f,%.6f\n",
                   err_angle_deg, omega_mes.x, omega_mes.y, omega_mes.z);
        }
        vTaskDelay(pdMS_TO_TICKS(2)) ;
    }
    
}
