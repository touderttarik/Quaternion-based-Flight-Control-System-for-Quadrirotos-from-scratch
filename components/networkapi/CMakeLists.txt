idf_component_register(
    SRCS "link_layer.c" "transport_layer.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event nvs_flash esp_netif log
)
