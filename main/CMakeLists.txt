idf_component_register(SRCS "02_lcd.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bsp_led
                    PRIV_REQUIRES lcd_ili9488
                    )
