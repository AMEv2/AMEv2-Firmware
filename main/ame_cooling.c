 /*********************************************************************************************************************
 * Filename  : ame_cooling.c
 * Author    : ChristiaanAlberts
 * Created on: 01 April 2024
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_cooling.c
 * @brief  	Cooling handling
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdatomic.h>

#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "pubsub.h"

#include "ame_cfg.h"

static const char *TAG = "cooling";

/**********************************************************************************************************************
* Module Preprocessor Constants
**********************************************************************************************************************/
#define MAX_COOLING_TIME_SECONDS            (20u * 60u)
#define MIN_COOLING_TIME_SECONDS            (5u * 60u)
#define COOLING_TIME_INCREMENT              (30u)

/**********************************************************************************************************************
* Module Preprocessor Macros
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Typedefs
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Variable Definitions
**********************************************************************************************************************/
//static esp_timer_handle_t cooling_end_timer;
static TimerHandle_t cooling_end_timer;
static atomic_uint_fast16_t tachCounter = 0;
static atomic_uint_fast16_t coolingCounter = MIN_COOLING_TIME_SECONDS;

/**********************************************************************************************************************
* Global Variable Definitions
**********************************************************************************************************************/
extern TaskHandle_t s_ame_cooling_task_handle;

/**********************************************************************************************************************
* Module Function Prototypes
**********************************************************************************************************************/
static void cooling_end_timer_callback(TimerHandle_t xTimer);
static void tach_isr_handler(void *arg);

/**********************************************************************************************************************
* Global Function Definitions
**********************************************************************************************************************/
void AME_COOLING_Task(void* pvArgs)
{
    ESP_LOGI(TAG, "Init");

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << CONFIG_IO_PIN_FANS) | (1ULL << CONFIG_IO_PIN_PUMP);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_IO_PIN_PUMP_TACH);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for home switch gpio pin
    gpio_isr_handler_add(CONFIG_IO_PIN_PUMP_TACH, tach_isr_handler, NULL);

    //ESP_ERROR_CHECK(esp_timer_create(&cooling_end_timer_args, &cooling_end_timer));
    cooling_end_timer = xTimerCreate(/* Just a text name, not used by the RTOSkernel. */
                        "cooling_end_timer",
                        /* The timer period in ticks, must be greater than 0. */
                        1,
                        /* The timers will auto-reload themselves when they expire. */
                        pdFALSE,
                        /* The ID can used to store a count of the number of times the timer has expired, which is initialised to 0. */
                        (void *)0,
                        /* Each timer calls the same callback when it expires. */
                        cooling_end_timer_callback);

    for(;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uint16_t rpm = tachCounter * 30;                // tachCounter / 2 * 60
        tachCounter = 0;
        PUB_DBL_FL("ame.cooling.pump_rpm", rpm, FL_STICKY);
    }
}

void AME_COOLING_Start(void)
{
    ESP_LOGI(TAG, "Start");
    // Turn fans and pump on
    gpio_set_level(CONFIG_IO_PIN_FANS, 1);
    gpio_set_level(CONFIG_IO_PIN_PUMP, 1);
    
    ESP_LOGI(TAG, "(Re)Start cooling Timer %f min", coolingCounter/60.0f);
    xTimerChangePeriod(cooling_end_timer, (coolingCounter * 1000u) / portTICK_PERIOD_MS, 100);

    coolingCounter += COOLING_TIME_INCREMENT;
    if (coolingCounter > MAX_COOLING_TIME_SECONDS)
    {
        coolingCounter = MAX_COOLING_TIME_SECONDS;
    }
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/
static void cooling_end_timer_callback(TimerHandle_t xTimer)
{
    // Turn off fans and pump
    gpio_set_level(CONFIG_IO_PIN_FANS, 0);
    gpio_set_level(CONFIG_IO_PIN_PUMP, 0);

    coolingCounter = MIN_COOLING_TIME_SECONDS;
}

static void IRAM_ATTR tach_isr_handler(void *arg)
{
    tachCounter++;
}

/*************** END OF FUNCTIONS ************************************************************************************/
