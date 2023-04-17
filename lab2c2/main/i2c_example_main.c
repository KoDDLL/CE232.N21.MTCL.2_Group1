#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <sdkconfig.h>
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "UIT.h"

static const char *TAG = "Test-I2C";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  
#define RW_TEST_LENGTH 128               
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 

#define I2C_MASTER_SCL_OLED 4            
#define I2C_MASTER_SDA_OLED 5            
#define I2C_MASTER_NUM_OLED I2C_NUMBER(0)
#define I2C_MASTER_FREQ_HZ 100000        
#define I2C_MASTER_TX_BUF_DISABLE 0            
#define I2C_MASTER_RX_BUF_DISABLE 0   

#define ESP_SLAVE_ADDR_OLED 0x3C 
#define WRITE_BIT 0 
#define READ_BIT 1 
#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                       
#define ACK_VAL 0x0                             
#define NACK_VAL 0x1                            

/**
 * @brief Test code to write esp-i2c-slave
 *   
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave
(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    // STEP #1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // STEP #2
    i2c_master_start(cmd);
    // STEP #3
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR_OLED << 1) | WRITE_BIT, ACK_CHECK_EN);
    // STEP #4
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    // STEP #5
    i2c_master_stop(cmd);
    // STEP #6
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    // STEP #7
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM_OLED;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_OLED;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_OLED;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, 
	conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); 
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); 

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void ssd1306_changePage(uint8_t current_page)
{
	i2c_cmd_handle_t cmd;

	uint8_t cur_page = current_page;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, 0x10, true); // reset line - choose line --> 0
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); 

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void GetHigh(uint8_t *Data, uint8_t *High){
	uint8_t k = 0;
	for (uint8_t i = 0; i < 16; i ++){
		if (i % 2 == 0){
			High[k] = Data[i];
			k++;
		}
	}
	k = 0;
}

void GetLow(uint8_t *Data, uint8_t *Low){
	uint8_t k = 0;
	for (uint8_t i = 0; i < 16; i ++){
		if (i % 2 != 0){
			Low[k] = Data[i];
			k++;
		}
	}
	k = 0;
}

void ssd1306_writeData(uint8_t *Data, uint8_t Len){
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte
			(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, Data, Len, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_text(const void *arg_text) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = 0;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, 0x10, true); // reset line - choose line --> 0
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte
			(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte
			(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
            
		}
	}

	vTaskDelete(NULL);
}
void task_ssd1306_display_clear(void *ignore) {
	i2c_cmd_handle_t cmd;

	uint8_t clear[128];
	for (uint8_t i = 0; i < 128; i++) {
		clear[i] = 0;
	}
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte
		(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C Master"); 
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Initialize successful");
    ESP_LOGI(TAG, "Initialize OLED");
    ssd1306_init();
    xTaskCreate(&task_ssd1306_display_clear, "ssd1306_display_clear",
	  2048, NULL, 6, NULL);
    vTaskDelay(100/portTICK_PERIOD_MS);
    xTaskCreate(&task_ssd1306_display_text, "ssd1306_display_text",  2048,
		(void *)"19521440\n19521703\n1952xxxx\n1952xxxx", 6, NULL);
    
    ESP_LOGI(TAG, "FINISH");
    vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_RATE_MS);
}