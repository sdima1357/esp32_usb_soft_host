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
struct USBMessage
{
	uint8_t src;
	uint8_t len;
	uint8_t  data[0x8];
};
static xQueueHandle usb_mess_Que = NULL;

void IRAM_ATTR timer_group0_isr(void *para)
{
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	//taskENTER_CRITICAL();
	usb_process();
	//taskEXIT_CRITICAL();
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
}
void usbMess(uint8_t src,uint8_t len,uint8_t *data)
{
	struct  USBMessage msg;
	msg.src = src;
	msg.len = len<0x8?len:0x8;
	for(int k=0;k<msg.len;k++)
	{
		msg.data[k] = data[k];
	}
	xQueueSend(usb_mess_Que, ( void * ) &msg,(TickType_t)0);
}

void kscan0()
{
	struct USBMessage msg;
        if(xQueueReceive(usb_mess_Que, &msg, 0)) 
	{
	    printf("\n%02x %02x ",msg.src,msg.len);
	    int unum= msg.src / 4;
            for(int k=0;k<msg.len;k++)
	   {
		   printf("%02x ",msg.data[k]);
	   }
	}
}
void test_delay1() {__asm__ (" nop");  }
void test_delay2() {__asm__ (" nop"); __asm__ (" nop"); }
void test_delay3() {__asm__ (" nop"); __asm__ (" nop"); __asm__ (" nop"); }
void stest()
{
    const char *test = "I am some test content to put in the heap";
    char buf[64];
    memset(buf, 0xEE, 64);
    strlcpy(buf, test, 64);

    char *a = malloc(64);
    memcpy(a, buf, 64);
    // move data from 'a' to IRAM
    char *b = heap_caps_realloc(a, 64, MALLOC_CAP_EXEC);
   printf("b=%p\n",b);
    printf("a=%p\n",a);	
    //~ assert((a-b)!=0);
    //~ assert(heap_caps_check_integrity(MALLOC_CAP_INVALID, true));
    //TEST_ASSERT_EQUAL_HEX32_ARRAY(buf, b, 64 / sizeof(uint32_t));

    // Move data back to DRAM
    char *c = heap_caps_realloc(b, 48, MALLOC_CAP_8BIT);
    printf("c=%p\n",c);	
    //~ TEST_ASSERT_NOT_NULL(c);
    //~ TEST_ASSERT_NOT_EQUAL(b, c);
    //~ TEST_ASSERT(heap_caps_check_integrity(MALLOC_CAP_INVALID, true));
    //~ TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, c, 48);

    free(c);
}


#if CONFIG_IDF_TARGET_ESP32C3
#define BLINK_GPIO 18
#define DP_P   6
#define DM_P  8

#define DP_P1  -1
#define DM_P1  -1
#else
#define BLINK_GPIO 22
#define DP_P   16
#define DM_P  17

#define DP_P1  -1
#define DM_P1  -1
#endif


#define DP_P2  -1
#define DM_P2  -1
#define DP_P3  -1
#define DM_P3  -1

//~ #define DP_P  16
//~ #define DM_P  17

//~ #define DP_P1  22
//~ #define DM_P1  23

//~ #define DP_P2  18
//~ #define DM_P2  19

//~ #define DP_P3  13
//~ #define DM_P3  15

int64_t get_system_time_us() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000000LL + (tv.tv_usec ));
}

void led(int on_fff)
{
			gpio_set_level(BLINK_GPIO, on_fff);
}
void setDelay(uint8_t ticks);

void timer_task(void *pvParameter)
{
	    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    setDelay(4);
    stest();
    usb_mess_Que  = xQueueCreate(10,sizeof(struct USBMessage));
    initStates(DP_P,DM_P,DP_P1,DM_P1,DP_P2,DM_P2,DP_P3,DM_P3);
  //  initStates(DP_P,DM_P,-1,-1,-1,-1,-1,-1);
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);


    int timer_idx = TIMER_0;
    double timer_interval_sec = TIMER_INTERVAL0_SEC;

     timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,(void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, timer_idx);
    //~ {
	//~ printf("test_delay1\n");
	//~ uint8_t *pnt = (uint8_t *)test_delay1;
	//~ for(int k=0;k<10;k++)
	//~ {
		//~ printf("%x\n",pnt[k]);
	//~ }
   //~ }
    //~ {
	//~ printf("test_delay2\n");
	//~ uint8_t *pnt = (uint8_t *)test_delay2;
	//~ for(int k=0;k<10;k++)
	//~ {
		//~ printf("%x\n",pnt[k]);
	//~ }
   //~ }
    //~ {
	//~ printf("test_delay3\n");
	//~ uint8_t *pnt = (uint8_t *)test_delay3;
	//~ for(int k=0;k<10;k++)
	//~ {
		//~ printf("%x\n",pnt[k]);
	//~ }
   //~ }
#define  DEBUG_P
    while(1)
    {
	   // printState();
	    kscan0();
#ifdef DEBUG_P
	    printState();
#endif	    
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
