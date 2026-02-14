#include "transport_layer.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "stdint.h"

#define TAG "Drone UDP"
#define LOCAL_BUFF_SIZE 512

void transport_task(void* pvParameters){

    (void)pvParameters;

    char localbuff[LOCAL_BUFF_SIZE];
    // local_addr: "My local endpo uint8_t" -> the local IP/port the tcp_socket is bound to.
    // In TCP server: used by bind() + listen() to choose the listening port.   
    // In UDP: used by bind() to choose the receive port.
    struct sockaddr_in local_addr = {0};

    local_addr.sin_family = AF_INET ;
    local_addr.sin_addr.s_addr = inet_addr(DRONE_ADDR);
    local_addr.sin_port = htons(DRONE_TCP_PORT) ;


    // peer_addr: "Remote endpouint8_t (destination)" -> the remote IP/port we want to reach.
    // In TCP client: passed to connect() to establish the connection.
    // In UDP: passed to sendto() as the destination.
    struct sockaddr_in peer_addr = {0}; //Not useful in the case of the server (Drone)

    // from_addr: "Remote endpouint8_t (who connected / who sent)" -> output filled by the stack.
    // In TCP server: accept() can fill it with the client's IP/port (who connected).
    // In UDP: recvfrom() fills it with the sender's IP/port (who sent this datagram).
    // Note: after TCP is connected, you usually don't need this struct for each recv()/send().
    struct sockaddr_in from_addr = {0};

    uint8_t tcp_socket = socket(AF_INET,SOCK_STREAM,0) ;//This will be used for creating a TCP connection, and this connection will be used for the critical messages.
    uint8_t udp_socket = socket(AF_INET,SOCK_DGRAM, 0) ;
    uint8_t new_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);//I don't know if it's up to me to create this one or it is the stack's job
    uint8_t error_code ;
    
    //create a tcp connection with the client (accept incoming connection)
    socklen_t tcp_sock_len = sizeof(local_addr) ;
    error_code = bind(tcp_socket,(struct sockaddr *) &local_addr, tcp_sock_len);
    if(error_code < 0){
        ESP_LOGE(TAG, "Binding of TCP socket failed, errno = %d", error_code);
    }
    else{
        ESP_LOGI(TAG, "Binding of TCP socket successful.");
    }
    error_code = listen(tcp_socket, 1);
    //What should I put here ?

    error_code = accept(tcp_socket, REMOTE_ADDR, tcp_sock_len);
    if(error_code < 0){
        ESP_LOGE(TAG, "TCP connection error between Drone and Remote. errno= %d", error_code);
    }
    else{
        ESP_LOGI(TAG, "TCP connection success between Drone and Remote.");
    }




    /*socklen_t socklen2 = sizeof(from_addr);
    uint8_t err = recvfrom(udp_socket, localbuff, LOCAL_BUFF_SIZE, 0,(struct sockaddr *)&from_addr, &socklen2) ;
    vTaskDelay(pdMS_TO_TICKS(1000));//Necessary because the WiFi doesn't initialize fast enough
    if( err<0 && hello_sem != NULL ){
        ESP_LOGE(TAG,"Data reception error. errno=%d",err);
    }
    else if(err > 0 && hello_sem != NULL){
        ESP_LOGI(TAG,"Message received from Remote !") ;
        xSemaphoreGive(hello_sem);
    }
*/

    closesocket(tcp_socket);
    vTaskDelete(NULL);
    
}
