#include <stdio.h>
#include "bsp_led.h"
#include "lcd.h"

void app_main(void)
{
    led_blink_task_create();
    lcd_test_task_create();
}
