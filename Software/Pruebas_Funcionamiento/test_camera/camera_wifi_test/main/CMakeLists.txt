idf_component_register(SRCS "main.c" "arducam_ov2640.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server esp_wifi nvs_flash driver esp_netif esp_driver_gpio esp_driver_spi esp_driver_i2c wpa_supplicant
                    EMBED_TXTFILES "index.html")
