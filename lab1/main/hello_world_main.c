#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#define signal 4
void app_main(void *pvParameter)
{
 gpio_pad_select_gpio(signal);
 gpio_set_direction(signal, GPIO_MODE_OUTPUT);
 while(1)
 {
printf("turning off the LED\n");
 gpio_set_level(signal, 0);
 vTaskDelay(1000/portTICK_PERIOD_MS);

printf("turning on the LED\n");
 gpio_set_level(signal, 1);
 vTaskDelay(1000/portTICK_PERIOD_MS);
 } }