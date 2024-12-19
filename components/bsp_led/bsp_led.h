#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_GPIO  2
#define BLINK_PERIOD 3000 /* ms */

#define STACK_SIZE 1024

void configure_led(void);
void blink_led(void);
uint8_t get_led_state(void);
void led_blink_task_create( void );

#ifdef __cplusplus
}
#endif

#endif