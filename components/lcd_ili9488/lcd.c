#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lcd.h"

void lcd_init(void);
void LCD_WR_DATA(uint8_t dat);
void LCD_WR_REG(uint8_t cmd);
void LCD_DisplayOn();
void LCD_DisplayOff();
uint16_t LCD_Read_ID();
void ili9488_read_color();
void lcd_clear(uint32_t color);
void fill_window(uint32_t color, int x1, int y1, int x2, int y2);
uint32_t randomNum(uint16_t right_edge);

static const char *TAG = "LCD";
spi_device_handle_t spi;

void lcd_test_task_code(void *pvParameters)
{
    lcd_init();

    uint16_t id = LCD_Read_ID();
    ESP_LOGI(TAG, "Lcd driver id: 0x%02X 0x%02X", (uint8_t)id>>8, (uint8_t)id&0xFF);
 
    lcd_clear(RED);
    ili9488_read_color();

    uint32_t color[7] = {MAGENTA, BLUE, BRRED, GREEN, YELLOW, GRAY, CYAN};

    ESP_LOGI(TAG, "lcdTestTaskCode start!");
    for( ;; )
    {   
        uint32_t x = randomNum(LCD_W);uint32_t y = randomNum(LCD_H);
        fill_window(color[randomNum(7)], x, y, x+randomNum(100),y+randomNum(100));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lcd_test_task_create( void )
{
    xTaskCreate(lcd_test_task_code, "LCD_TEST_TASK", 2048, NULL, 2, NULL);
}


void init_spi()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 * 20 // 4K
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M,  // Clock out at 10 MHz
        .mode = 0,                              // SPI mode 0
        .spics_io_num = PIN_NUM_CS,             // CS pin
        .queue_size = 7,                        // We want to be able to queue 7 transactions at a time
        // .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void LCD_WR_REG(uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    LCD_DC_CLR(); // 写命令
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void LCD_WR_REG_WITH_CS_ACTIVE(uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    LCD_DC_CLR(); // 写命令
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void LCD_WR_DATA(uint8_t dat)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &dat;               //The data is the cmd itself
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void LCD_WR_DATA_WITH_CS_ACTIVE(uint8_t dat)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &dat;               //The data is the cmd itself
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void LCD_WR_DATA_X(uint8_t *data, uint32_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8 * size;                     //Command is 8 bits
    t.tx_buffer = data;               //The data is the cmd itself
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

uint8_t LCD_RD_DATA()
{
    uint8_t data;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.rx_buffer = &data;               //The data is the cmd itself
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.

    return data;
}

uint8_t LCD_RD_DATA_WITH_CS_ACTIVE()
{
    uint8_t data;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.rx_buffer = &data;               //The data is the cmd itself
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.

    return data;
}

void LCD_RD_DATA_X(uint8_t *data, uint32_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));         //Zero out the transaction
    t.length = 8 * size;                     //Command is 8 bits
    t.rx_buffer = data;               //The data is the cmd itself
    LCD_DC_SET(); // 写数据
    esp_err_t ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

//读显示屏 ID
uint16_t LCD_Read_ID()
{
    uint8_t temp = 0;    
    uint8_t reg = 0xD3;
    uint16_t id = 0;

    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);

#if 0
    LCD_WR_REG_WITH_CS_ACTIVE(0xFB); 	
    LCD_WR_DATA_WITH_CS_ACTIVE(0x80);
	LCD_WR_REG_WITH_CS_ACTIVE(reg);
    temp = LCD_RD_DATA_WITH_CS_ACTIVE();        //dummy read 
    ESP_LOGI(TAG, "temp data: 0x%02X", temp);

    LCD_WR_REG_WITH_CS_ACTIVE(0xFB); 	
    LCD_WR_DATA_WITH_CS_ACTIVE(0x81);	
	LCD_WR_REG_WITH_CS_ACTIVE(reg);
	temp = LCD_RD_DATA_WITH_CS_ACTIVE();        //dummy read 
    ESP_LOGI(TAG, "temp data: 0x%02X", temp);
		
#endif
	LCD_WR_REG(0xFB); 	
    LCD_WR_DATA(0x82);
	LCD_WR_REG_WITH_CS_ACTIVE(reg);
	temp = LCD_RD_DATA();   	//读取94
    ESP_LOGI(TAG, "temp data: 0x%02X", temp);								   
	id = temp;
    id <<= 8;

	LCD_WR_REG(0xFB); 	
    LCD_WR_DATA(0x83);
	LCD_WR_REG_WITH_CS_ACTIVE(reg);
	temp = LCD_RD_DATA();  	//读取88 	
    ESP_LOGI(TAG, "temp data: 0x%02X", temp);
    id |= temp;

    // Release bus
    spi_device_release_bus(spi);

	LCD_WR_REG(0xFB); 	
    LCD_WR_DATA(0x00);
		
	return id;
}

void lcd_set_window(int x1, int y1, int x2, int y2) {
    uint8_t data[4];

    // 设置列地址（x 坐标）
    LCD_WR_REG(0x2A);
    data[0] = (x1 >> 8) & 0xFF;
    data[1] =  x1 & 0xFF;
    data[2] = (x2 >> 8) & 0xFF; 
    data[3] =  x2 & 0xFF;       
    LCD_WR_DATA_X(data, 4);
    // LCD_WR_DATA(data[0]);
    // LCD_WR_DATA(data[1]);    
    // LCD_WR_DATA(data[2]);
    // LCD_WR_DATA(data[3]);

    // 设置行地址（y 坐标）
    LCD_WR_REG(0x2B); 
    data[0] = (y1 >> 8) & 0xFF;
    data[1] =  y1 & 0xFF;
    data[2] = (y2 >> 8) & 0xFF; 
    data[3] =  y2 & 0xFF;       
    LCD_WR_DATA_X(data, 4);
    // LCD_WR_DATA(data[0]);
    // LCD_WR_DATA(data[1]);    
    // LCD_WR_DATA(data[2]);
    // LCD_WR_DATA(data[3]);
}

void lcd_clear(uint32_t color)
{
    lcd_set_window(0, 0, LCD_W - 1, LCD_H - 1);
    /* GRAM WR*/
    LCD_WR_REG(0x2C);

    // 准备颜色数据
    uint8_t r, g, b;
    r = (uint8_t)((color >> 12) & 0x3f) << 2;
    g = (uint8_t)((color >> 6) & 0x3f) << 2;
    b = (uint8_t)((color) & 0x3f) << 2;

    unsigned scale = 60;
    uint8_t *row_color_bytes = malloc(3 * LCD_W * scale);

    for(uint32_t i = 0; i < (LCD_W * scale); i++) {
        row_color_bytes[3*i] = r;
        row_color_bytes[3*i+1] = g;
        row_color_bytes[3*i+2] = b;
    }

    for(uint32_t i = 0; i < (LCD_H / scale); i++){
        // LCD_WR_DATA(r);
        // LCD_WR_DATA(g);
        // LCD_WR_DATA(b);
        LCD_WR_DATA_X(row_color_bytes, 3 * LCD_W * scale);
	}

    free(row_color_bytes);
}

void fill_window(uint32_t color, int x1, int y1, int x2, int y2)
{
    lcd_set_window(x1, y1, x2, y2);
    /* GRAM WR*/
    LCD_WR_REG(0x2C);

    // 准备颜色数据
    uint8_t r, g, b;
    r = (uint8_t)((color >> 12) & 0x3f) << 2;
    g = (uint8_t)((color >> 6) & 0x3f) << 2;
    b = (uint8_t)((color) & 0x3f) << 2;

    uint32_t width = x2 - x1 + 1;
    uint32_t height = y2 - y1 + 1;

    unsigned scale = 1;
    uint8_t *row_color_bytes = malloc(3 * width * scale);

    for(uint32_t i = 0; i < (width * scale); i++) {
        row_color_bytes[3*i] = r;
        row_color_bytes[3*i+1] = g;
        row_color_bytes[3*i+2] = b;
    }

    for(uint32_t i = 0; i < (height / scale); i++){
        // LCD_WR_DATA(r);
        // LCD_WR_DATA(g);
        // LCD_WR_DATA(b);
        LCD_WR_DATA_X(row_color_bytes, 3 * width * scale);
	}

    free(row_color_bytes);
}

void ili9488_read_color() {
    uint8_t color_data[4] = {0};
    
    lcd_set_window(0, 0, 10, 10);

    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);
    LCD_WR_REG_WITH_CS_ACTIVE(0x2E);

    LCD_RD_DATA_X(color_data, 4);

    // Release bus
    spi_device_release_bus(spi);

    ESP_LOGI(TAG, "color data: 0x%02X 0x%02X 0x%02X 0x%02X", color_data[0], color_data[1], color_data[2], color_data[3]);
}

void lcd_init(void)
{
    esp_rom_gpio_pad_select_gpio(PIN_NUM_RESET);
    esp_rom_gpio_pad_select_gpio(PIN_NUM_DC);
    esp_rom_gpio_pad_select_gpio(PIN_NUM_BACKLIGHT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BACKLIGHT, GPIO_MODE_OUTPUT);

    /* Backlight on */
    gpio_set_level(PIN_NUM_BACKLIGHT, 1);

    init_spi();

    /* LCD reset */
    LCD_RES_SET();
	LCD_DELAY(1);
    LCD_RES_CLR();
	LCD_DELAY(10);	
    LCD_RES_SET();
	LCD_DELAY(120);

    /************* Start Initial Sequence **********/
    LCD_WR_REG(0XF7);    	
    LCD_WR_DATA(0xA9); 	
    LCD_WR_DATA(0x51); 	
    LCD_WR_DATA(0x2C); 	
    LCD_WR_DATA(0x82);

    LCD_WR_REG(0XEC);    	
    LCD_WR_DATA(0x00); 	
    LCD_WR_DATA(0x02); 	
    LCD_WR_DATA(0x03); 	
    LCD_WR_DATA(0x7A);

    LCD_WR_REG(0xC0); 	
    LCD_WR_DATA(0x13); 	
    LCD_WR_DATA(0x13); 	
        
    LCD_WR_REG(0xC1); 	
    LCD_WR_DATA(0x41); 	
        
    LCD_WR_REG(0xC5); 	
    LCD_WR_DATA(0x00); 	
    LCD_WR_DATA(0x28); 	
    LCD_WR_DATA(0x80);
        
    LCD_WR_REG(0xB0);    	
    LCD_WR_DATA(0x00);
        
        
    LCD_WR_REG(0xB1);   //Frame rate 70HZ  	
    LCD_WR_DATA(0xB0);
    LCD_WR_DATA(0x11);	
        
    LCD_WR_REG(0xB4); 	
    LCD_WR_DATA(0x02);   	
        
    LCD_WR_REG(0xB6); //RGB/MCU Interface Control	
    LCD_WR_DATA(0x02);   //MCU	
    LCD_WR_DATA(0x22); 

    LCD_WR_REG(0xB7); 	
    LCD_WR_DATA(0xc6); 

    LCD_WR_REG(0xBE); 	
    LCD_WR_DATA(0x00); 
    LCD_WR_DATA(0x04);	
        
    LCD_WR_REG(0xE9); 	
    LCD_WR_DATA(0x00);

    LCD_WR_REG(0xF4); 	
    LCD_WR_DATA(0x00); 
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x0f);	
        
    LCD_WR_REG(0xE0); 	
    LCD_WR_DATA(0x00); 	
    LCD_WR_DATA(0x04); 	
    LCD_WR_DATA(0x0E); 	
    LCD_WR_DATA(0x08); 	
    LCD_WR_DATA(0x17); 	
    LCD_WR_DATA(0x0A); 	
    LCD_WR_DATA(0x40); 	
    LCD_WR_DATA(0x79); 	
    LCD_WR_DATA(0x4D); 	
    LCD_WR_DATA(0x07); 	
    LCD_WR_DATA(0x0E); 	
    LCD_WR_DATA(0x0A); 	
    LCD_WR_DATA(0x1A); 	
    LCD_WR_DATA(0x1D); 	
    LCD_WR_DATA(0x0F);  	
        
    LCD_WR_REG(0xE1); 	
    LCD_WR_DATA(0x00); 	
    LCD_WR_DATA(0x1B); 	
    LCD_WR_DATA(0x1F); 	
    LCD_WR_DATA(0x02); 	
    LCD_WR_DATA(0x10); 	
    LCD_WR_DATA(0x05); 	
    LCD_WR_DATA(0x32); 	
    LCD_WR_DATA(0x34); 	
    LCD_WR_DATA(0x43); 	
    LCD_WR_DATA(0x02); 	
    LCD_WR_DATA(0x0A); 	
    LCD_WR_DATA(0x09); 	
    LCD_WR_DATA(0x33); 	
    LCD_WR_DATA(0x37); 	
    LCD_WR_DATA(0x0F); 


    LCD_WR_REG(0xF4);      
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x0f);	
        
    LCD_WR_REG(0x36); 	
    LCD_WR_DATA(0x08); 	
        
    LCD_WR_REG(0x3A);  //Interface Mode Control	
    LCD_WR_DATA(0x66);  //0x66 18bit; 0x55 16bit			
        
    LCD_WR_REG(0x20); 	
    LCD_WR_REG(0x11); 	
    LCD_DELAY(120); 	
    LCD_WR_REG(0x29); 	
    LCD_DELAY(50); 
}

//LCD开启显示
void LCD_DisplayOn(void)
{					   
	LCD_WR_REG(0X29);	//开启显示
}	 
//LCD关闭显示
void LCD_DisplayOff(void)
{	   
	LCD_WR_REG(0X28);	//关闭显示
}   

uint32_t randomNum(uint16_t right_edge)
{
    return esp_random() % right_edge;
}

