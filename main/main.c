#include "sensors.h"
#include "mahony.h"
#include "plot.h"
#include "pid.h"
#include "freertos/FreeRTOS.h"

#define SCL_GPIO 16
#define SDA_GPIO 17
#define I2C_FREQ_HZ 400000

void app_main(void)
{
    plot_state_t plot_state;
    plot_init(&plot_state, 10, 57.2957795f);
    
    // Initialize I2C Bus and configure the MPU to +/- 8g and 1000 deg/s
    mpu6050_init(SCL_GPIO, SDA_GPIO, I2C_FREQ_HZ);
    // Initialize Mahony filter state
    mahony_t mahony ;
    pid_struct_t pid ;
    mahony_init(&mahony);
    pid_init(&pid);
    // Sampling period: 2ms for 500Hz loop frequency
    float delta_t = 0.002f;
    float inverse_delta_t = 1/delta_t ; //for the sake of optimization we calculate this inverse once and use it at pid() call
    sensor_data_t sensors = {0};
    
    // Main control loop
    for(;;) {
        
        read_sensors(&sensors);
        mahony_update(&mahony, &sensors, delta_t) ;
        //plot_emit(&plot_state, &mahony, &sensors);
        pid_update(&mahony, &pid, delta_t, inverse_delta_t, &sensors) ;
        //Here starts the PID journey
        
        
        // Delay 2ms to maintain 500Hz control loop frequency
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
