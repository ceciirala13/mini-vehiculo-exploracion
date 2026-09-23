idf_component_register(SRCS "arducam_ov2640.c" "main.cpp" "uart_bridge.cpp" "bme690.c" "BME690_SensorAPI/bme69x.c"
                    INCLUDE_DIRS "." "BME690_SensorAPI"
                    REQUIRES esp_driver_uart manejo servo driver)
