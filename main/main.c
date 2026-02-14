#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "link_layer.h"
#include "transport_layer.h"
#include "sensors.h"
#include "motors.h"
#include "mahony.h"
#include "pid.h"
#include "mix.h"



#define SCL_GPIO 22
#define SDA_GPIO 21
#define I2C_FREQ_HZ 400000

#define PWM1_GPIO 16
#define PWM2_GPIO 18
#define PWM3_GPIO 13
#define PWM4_GPIO 14

static const char *TAG = "DRONE MAIN APP";
SemaphoreHandle_t hello_sem = NULL;
SemaphoreHandle_t arm_seq_sem = NULL;

static const quat_t k_level_setpoint = {1.0f, 0.0f, 0.0f, 0.0f};
static const float k_throttle = 0.0f;



void app_main(void)
{
    ESP_LOGI(TAG, "Boot");
    wifi_init_softap();
    // INITIALISATION obligatoire avant de créer la tâche
    hello_sem = xSemaphoreCreateBinary() ;
    arm_seq_sem = xSemaphoreCreateBinary() ;
    if (hello_sem != NULL) {
        xTaskCreate(transport_task, "udp_server_task", 4096, NULL, 5, NULL);
        // Blocage du main jusqu'au "Give" dans udp_task.c
        ESP_LOGI(TAG, "Waiting for Hello Drone.");
        xSemaphoreTake(hello_sem, portMAX_DELAY);
        ESP_LOGI(TAG, "Le main continue !");
    }
    
    if(arm_seq_sem != NULL){
        xSemaphoreTake(arm_seq_sem, portMAX_DELAY);
    }

    mpu6050_init(SCL_GPIO, SDA_GPIO, I2C_FREQ_HZ);
    motors_init(PWM1_GPIO, PWM2_GPIO, PWM3_GPIO, PWM4_GPIO);

    mahony_t mahony;
    pid_struct_t pid;
    mix_t mix;
    mahony_init(&mahony);
    pid_init(&pid);

    const float delta_t = 0.002f;
    const float inverse_delta_t = 1.0f / delta_t;
    sensor_data_t sensors = {0};

    for (;;) {
        read_sensors(&sensors);
        mahony_update(&mahony, &sensors, delta_t);

        pid_update(&mahony, &pid, delta_t, inverse_delta_t, &sensors, k_level_setpoint);
        mix_update(&pid, &mix, k_throttle);
        write_motors(&mix);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
