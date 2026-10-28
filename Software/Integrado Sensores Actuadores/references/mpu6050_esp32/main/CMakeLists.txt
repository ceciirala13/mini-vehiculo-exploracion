idf_component_register(SRCS "main.c"
                        INCLUDE_DIRS "."
                        REQUIRES mpu6050 driver esp_driver_i2c)
