#pragma once

#include "freertos/semphr.h"


#define BUF_SIZE 1400
#define PACKETS_TO_SEND 1
#define DRONE_ADDR "192.168.4.1"
#define REMOTE_ADDR "192.168.4.2"
#define REMOTE_TCP_PORT 50000
#define REMOTE_UDP_PORT 50001
#define DRONE_TCP_PORT 50000
#define DRONE_UDP_PORT 50001

extern SemaphoreHandle_t hello_sem;
extern SemaphoreHandle_t arm_seq_sem;


void udp_client_task(void *pvParameters);
void udp_server_task(void *pvParameters);
