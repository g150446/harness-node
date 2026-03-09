idf_component_register(SRCS "main.c" "adpcm.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2s bt esp_netif esp_driver_uart)
