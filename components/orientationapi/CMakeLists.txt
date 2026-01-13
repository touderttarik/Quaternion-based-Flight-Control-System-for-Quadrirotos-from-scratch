idf_component_register(
    SRCS "pid.c" "matrix.c" "quaternions.c" "vectors.c" "mahony.c" "sensors.c" "plot.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
)
