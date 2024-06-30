 /*********************************************************************************************************************
 * Filename  : ame_dropgate.c
 * Author    : ChristiaanAlberts
 * Created on: 01 June 2024
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_dropgate.c
 * @brief  	drop gate control
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

// #include "pubsub.h"

static const char *TAG = "ame_dropgate";

/**********************************************************************************************************************
* Module Preprocessor Constants
**********************************************************************************************************************/
#define DROP_GATE_OUTPUT_MASK (1ULL << CONFIG_IO_PIN_DROP_GATE)

/**********************************************************************************************************************
* Module Preprocessor Macros
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Typedefs
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Variable Definitions
**********************************************************************************************************************/
static SemaphoreHandle_t dropGateSemaphore;

/**********************************************************************************************************************
* Global Variable Definitions
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Function Prototypes
**********************************************************************************************************************/


/**********************************************************************************************************************
* Global Function Definitions
**********************************************************************************************************************/
 void AME_DROPGATE_Init(void)
 {
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = DROP_GATE_OUTPUT_MASK;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    /* Create a mutex type semaphore. */
    dropGateSemaphore = xSemaphoreCreateMutex();
 }

void AME_DROPGATE_DropCase(void)
{
    if (dropGateSemaphore != NULL)
    {
        /* See if we can obtain the semaphore.  If the semaphore is not
        available wait 10 ticks to see if it becomes free. */
        if (xSemaphoreTake(dropGateSemaphore, (TickType_t)10) == pdTRUE)
        {
            /* We were able to obtain the semaphore and can now access the
            shared resource. */

            ESP_LOGI(TAG, "Dropping Case!");
            gpio_set_level(CONFIG_IO_PIN_DROP_GATE, 1);
            vTaskDelay(pdMS_TO_TICKS(500));

            gpio_set_level(CONFIG_IO_PIN_DROP_GATE, 0);

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive(dropGateSemaphore);
        }
        else
        {
            /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
        }
    }
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/


/*************** END OF FUNCTIONS ************************************************************************************/
