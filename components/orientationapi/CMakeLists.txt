idf_component_register(
    SRCS "mix.c" "pid.c" "matrix.c" "quaternions.c" "vectors.c" "mahony.c" "sensors.c" "plot.c" "motors.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_mcpwm
)
