idf_component_register(SRCS "lcd.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_spi
                    )