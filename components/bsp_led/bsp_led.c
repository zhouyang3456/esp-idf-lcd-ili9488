#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bsp_led.h"

static const char *TAG = "blink";
static uint8_t s_led_state = 0;

void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
    /* Toggle led state */
    s_led_state = !s_led_state;
}

void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

uint8_t get_led_state(void) 
{
    return s_led_state;
}

// Task to be created.
void vTaskCode( void * pvParameters )
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    for( ;; )
    {
        blink_led();
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
    }
}

// Function that creates a task.
void led_blink_task_create( void )
{
static uint8_t ucParameterToPass;
TaskHandle_t xHandle = NULL;

// Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
// must exist for the lifetime of the task, so in this case is declared static.  If it was just an
// an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
// the new task attempts to access it.
  xTaskCreate(vTaskCode, "LED_BLINK_TASK", 2048, NULL, 3, NULL);
}