#include <stdio.h>
#include "quaternions.h"
#include "vectors.h"
#include "matrix.h"
#include "mahony.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"

//But du filtre : estimer R et estimer b pour corriger le gyro

void app_main(void)

{
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
        .device_address = 0x68,
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
    uint8_t wakeupbuff[2] = {0x6B, 0x00};
    esp_err_t err_wr = i2c_master_transmit(dev_handle, wakeupbuff, 2, 1000);
    if (err_wr == ESP_OK) printf("Réveillé !\n");


    //variables mahony
    mahony_t mahony ;
    mahony_init(&mahony) ;
    float delta_t = 0.002 ; //mesure toutes les 0.002 seconde (boucle à 500 Hz)
    sensor_data_t sensors= {0};
    quat_t q_hat_dot = quat_make (0, 0, 0, 0) ;
    quat_t va_quat = quat_make(0, mahony.va.x, mahony.va.y, mahony.va.z);
    quat_t q_hat_conj = quat_make(0,0,0,0);//init à n'importe quoi
    quat_t quat_temp = quat_make(0,0,0,0) ;// sera utilisé pour la partie gauche du sandwich
    vec3_t va_hat = {0} ;
    quat_t va_hat_quat = quat_make(0,0,0,0) ;// exprimé le vecteur prédiction sous forme de quaternion
    matrix_t va_skew_matrix ; //pour éviter de faire un cross product à la formule longue et comliquée
    vec3_t omega_mes ; //erreur prédiction-mesure capeur (un vecteur normal au plan de va_hat et va)
    vec3_t omega ; //vitesse du gyro corrigée (inclut la lecture du gyro Omega_y, le bias prédit et l'erreur)
    quat_t q_pure_omega = quat_make(0, 0, 0, 0) ;//expression de omega sous forme de quaternion pur
    vec3_t bias_hat = {0} ;
    //variables de lecture de données
    uint8_t AccelReg = 0x3B ;
    uint8_t GyroReg = 0x43 ;
    int16_t xAccelraw = 0 ;
    int16_t yAccelraw = 0 ;
    int16_t zAccelraw = 0 ;
    int16_t xGyroraw = 0 ;
    int16_t yGyroraw = 0 ;
    int16_t zGyroraw = 0 ;
    uint8_t AccelData [6] ;
    uint8_t GyroData [6] ;
    for(;;){
        i2c_master_transmit_receive(dev_handle, &AccelReg, 1, AccelData, 6, 100);
        xAccelraw = (AccelData[0]<<8) | AccelData[1] ;
        yAccelraw = (AccelData[2]<<8) | AccelData[3] ;
        zAccelraw = (AccelData[4]<<8) | AccelData[5] ;
        sensors.accel.x    = xAccelraw / 16384.0f;
        sensors.accel.y    = yAccelraw / 16384.0f ;
        sensors.accel.z    = zAccelraw / 16384.0f ;
        i2c_master_transmit_receive(dev_handle, &GyroReg,1, GyroData, 6,100);
        xGyroraw  = (GyroData[0] << 8 ) | GyroData[1] ;
        yGyroraw  = (GyroData[2] << 8 ) | GyroData[3];
        zGyroraw  = (GyroData[4] << 8 ) | GyroData[5] ;
        sensors.gyro.x     =  (xGyroraw / 131.0f)*(2*3.14159/360) ;
        sensors.gyro.y     =  (yGyroraw / 131.0f)*(2*3.14159/360) ;
        sensors.gyro.z     =  (zGyroraw / 131.0f)*(2*3.14159/360) ;        
        
        
        //normaliser va
        mahony.va.x = sensors.accel.x ;
        mahony.va.y = sensors.accel.y ;
        mahony.va.z = sensors.accel.z ;
        vect_normalize(&mahony.va) ;
        
        //prédire la gravité va_hat
        q_hat_conj = quat_conj(mahony.q_hat) ;
        quat_temp = quat_mul(q_hat_conj, mahony.e3) ;
        va_hat_quat = quat_mul(quat_temp,mahony.q_hat);
        printf("va_hat x = %f | va_hat y = %f | va_hat_z = %f \n",va_hat_quat.x, va_hat_quat.y, va_hat_quat.z );
        //printf("va x = %f | va y = %f | va_z = %f \n",mahony.va.x, mahony.va.y, mahony.va.z);
        //calculer l'erreur omeag mes
        va_skew_matrix = get_skew_mat(mahony.va) ;
        va_hat.x = va_hat_quat.x ;
        va_hat.y = va_hat_quat.y ;
        va_hat.z = va_hat_quat.z ;
        omega_mes = mat_vec_prod(va_skew_matrix, va_hat) ;
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
        q_hat_dot.x = q_hat_dot.x/2;
        q_hat_dot.y = q_hat_dot.y/2;
        q_hat_dot.z = q_hat_dot.z/2;
        //intégrer
        mahony.q_hat.x = mahony.q_hat.x + q_hat_dot.x*delta_t ;
        mahony.q_hat.y = mahony.q_hat.y + q_hat_dot.y*delta_t ;
        mahony.q_hat.z = mahony.q_hat.z + q_hat_dot.z*delta_t ;
        mahony.q_hat   = quat_normalize(mahony.q_hat) ;
        vTaskDelay(pdMS_TO_TICKS(2)) ;
    }
    
}
