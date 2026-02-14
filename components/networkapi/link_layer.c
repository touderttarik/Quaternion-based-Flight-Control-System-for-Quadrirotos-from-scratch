#include "link_layer.h"
#include "esp_log.h"
#include <stdio.h>

#define TAG "Drone SoftAP"

/*
The steps to set up a Wi-Fi soft-AP using the ESP-IDF framework are as follows:

1-Create a new project based on the hello_world example.
2-Initialize the esp_netif library.
3-Start the standard event loop to handle Wi-Fi events.
4-Configure and launch a soft-AP, allowing devices to connect via Wi-Fi.
5-Verify event handling by monitoring the terminal for connection events.
*/





static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);



/*Espressif’s Wi-Fi component relies on an event loop to handle asynchronous events.
Therefore, we need to:

Start the default event loop.
Create and register an event handler function to process Wi-Fi events.
For now, this function will simply print the event_id.

To keep things clean, we will create a function called wifi_init_softap where we will 
encapsulate all the steps listed above required to start the soft-AP.*/
void wifi_init_softap(void){

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();  
    esp_netif_create_default_wifi_ap();
    

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = ESP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);

}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data){
    (void)arg;//we put this line to avoid compiler warnings
    (void)event_base;//same here
    (void)event_data;//and here
    switch(event_id){
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "SoftAP Start.");
            break;
        case WIFI_EVENT_AP_STOP :
            ESP_LOGI(TAG, "SoftAP Stop.") ;
            break ;
        case WIFI_EVENT_AP_STACONNECTED :
            ESP_LOGI(TAG,"Remote Connected !") ;
            break ;
    }
}