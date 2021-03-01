/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "hal/timer_ll.h"
/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/periph_defs.h"
#include "esp_log.h"

#include "usb_host.h"

void IRAM_ATTR timer_group0_isr(void *para)
{
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	usb_process();
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
}

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define DP_P  16
#define DM_P  17

#define DP_P1  22
#define DM_P1  23

void timer_task(void *pvParameter)
{
	    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    
    
    initStates(BLINK_GPIO,DP_P,DM_P,DP_P1,DM_P1,-1,-1,-1,-1);
  //  initStates(BLINK_GPIO,DP_P,DM_P,-1,-1,-1,-1,-1,-1);

    int timer_idx = TIMER_0;
    double timer_interval_sec = TIMER_INTERVAL0_SEC;

     timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,(void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, timer_idx);
    while(1)
    {
	    printState();
	    vTaskDelay(10 / portTICK_PERIOD_MS);
     };
    
}


void blink_task(void *pvParameter)
{
	
	
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    while(1) {
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(&timer_task, "timer_task", 4096, "task 0", 5, NULL,1);
    xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096, "task 1", 5, NULL,0);
   printf("Hello world end!\n");	
}
