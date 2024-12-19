#ifndef __LCD_H__
#define __LCD_H__

#define LCD_W 320
#define LCD_H 480

#define LCD_HOST VSPI_HOST

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

#define PIN_NUM_DC 4
#define PIN_NUM_RESET 15
#define PIN_NUM_BACKLIGHT 12

#define WHITE            0xFFFFF
#define BLACK         	 0x00000	  
#define BLUE         	 0x0003F  
#define BRED             0x3F03F
#define GRED 			 0x3FFC0
#define GBLUE			 0x00FFF
#define RED           	 0x3F000
#define MAGENTA       	 0x3F03F
#define GREEN         	 0x00FC0
#define CYAN          	 0x0FFFF
#define YELLOW        	 0x3FFC0
#define BROWN 			 0XBC40 
#define BRRED 			 0XFC07 
#define GRAY  			 0X8430 
 
#define LCD_CS_SET() gpio_set_level(PIN_NUM_CS,1);
#define LCD_CS_CLR() gpio_set_level(PIN_NUM_CS,0);
#define LCD_RES_SET() gpio_set_level(PIN_NUM_RESET,1);
#define LCD_RES_CLR() gpio_set_level(PIN_NUM_RESET,0);
#define LCD_DC_SET() gpio_set_level(PIN_NUM_DC,1);
#define LCD_DC_CLR() gpio_set_level(PIN_NUM_DC,0);
#define LCD_BACKLIGHT_SET() gpio_set_level(PIN_NUM_BACKLIGHT,1);
#define LCD_BACKLIGHT_CLR() gpio_set_level(PIN_NUM_BACKLIGHT,0);

#define LCD_DELAY(x) vTaskDelay(pdMS_TO_TICKS(x)); 

void lcd_test_task_create(void);

#endif