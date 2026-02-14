#pragma once

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <string.h>

#define ESP_WIFI_SSID "DRONE LINK"
#define ESP_WIFI_PASS "drone1234"
#define ESP_WIFI_CHANNEL 1
#define ESP_MAXIMUM_RETRY 5
#define ESP_MAX_STA_CONN 1 //The remote only 



void wifi_init_softap(void);
