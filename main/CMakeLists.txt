idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES orientationapi
                    PRIV_REQUIRES esp_driver_i2c)
