#include "sensors.h"
#include "mahony.h"
#include "plot.h"
#include "pid.h"
#include "mix.h"
#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCL_GPIO 22
#define SDA_GPIO 21
#define I2C_FREQ_HZ 400000
#define PWM1_GPIO 32//motor 1
#define PWM2_GPIO 25//motor 2
#define PWM3_GPIO 26//motor 3
#define PWM4_GPIO 33//motor 4

void app_main(void)
{
    plot_state_t plot_state;

    plot_state_t plot_ctrl_state;
    plot_init(&plot_state, 10, 57.2957795f);

    plot_ctrl_init(&plot_ctrl_state, 10, 57.2957795f);
    
    // Initialize I2C Bus and configure the MPU to +/- 8g and 1000 deg/s
    mpu6050_init(SCL_GPIO, SDA_GPIO, I2C_FREQ_HZ);

    //Initialize the MCPWM generator
    motors_init(PWM1_GPIO, PWM2_GPIO, PWM3_GPIO, PWM4_GPIO) ;

    // Initialize Mahony filter state
    mahony_t mahony ;
    pid_struct_t pid ;
    mix_t mix ;
    mahony_init(&mahony);
    pid_init(&pid);
    // Sampling period: 2ms for 500Hz loop frequency
    float delta_t = 0.002f;
    float inverse_delta_t = 1/delta_t ; //for the sake of optimization we calculate this inverse once and use it at pid() call
    sensor_data_t sensors = {0};
    
    // Main control loop
    for(;;) {
        
        //Read the control commands from the remote
//      read_remote() ;
        read_sensors(&sensors);
        //Here occurs the magic of the Mahony filter
        mahony_update(&mahony, &sensors, delta_t) ;
        //plot_emit(&plot_state, &mahony, &sensors);//Plot Mahony data
        
        pid_update(&mahony, &pid, delta_t, inverse_delta_t, &sensors) ;
        
        mix_update(&pid, &mix) ;
        
        write_motors(&mix) ;
        
        
        plot_ctrl_emit(&plot_ctrl_state, &pid, &mix);
        // Delay 2ms to maintain 500Hz control loop frequency
        //I have to pay attention to the real duration of my loop.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
