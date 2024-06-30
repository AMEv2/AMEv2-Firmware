/*********************************************************************************************************************
 * Filename  : ame_anneal.c
 * Author    : ChristiaanAlberts
 * Created on: 4 June 2024
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_anneal.c
 * @brief  	Main anneal task
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "pubsub.h"

#include "ame_dropgate.h"
#include "ame_anneal.h"
#include "ame_stepper.h"
#include "ame_cooling.h"
#include "ame_adc.h"
#include "ame_cfg.h"

static const char *TAG = "anneal";

/**********************************************************************************************************************
* Module Preprocessor Constants
**********************************************************************************************************************/
#define EVENT_START_ANNEAL      (0x01)
#define EVENT_STOP_ANNEAL       (0x02)


/**********************************************************************************************************************
* Module Preprocessor Macros
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Typedefs
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Variable Definitions
**********************************************************************************************************************/
static uint16_t totalCases = 0;
static uint16_t casesDone = 0;
static uint16_t annealTime_ms = 0;
static uint16_t remainingCases = 0;

static esp_timer_handle_t anneal_timer;
static esp_timer_handle_t anneal_timer_ui_update;
static esp_timer_handle_t anneal_timer_zvs_power;

static atomic_bool isAnnealing = false;
static atomic_bool annealSessionActive = false;

/**********************************************************************************************************************
* Global Variable Definitions
**********************************************************************************************************************/
extern TaskHandle_t s_ame_anneal_task_handle;

/**********************************************************************************************************************
* Module Function Prototypes
**********************************************************************************************************************/
static void ame_anneal_Run(void);
static void ame_anneal_End(void);

static void ame_anneal_timer_callback(void *arg);
static void ame_anneal_timer_callback_ui_update(void *arg);
static void ame_anneal_timer_callback_zvs_power(void *arg);


/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void AME_ANNEAL_Task(void *pvArgs)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << CONFIG_IO_PIN_ZVS_CTRL | 1ULL << CONFIG_IO_PIN_ZVS_ON_OFF);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 0);

    // use low precision timer for timing display
    const esp_timer_create_args_t periodic_timer_args_ui_update = {
        .callback = &ame_anneal_timer_callback_ui_update,
        .dispatch_method = ESP_TIMER_TASK,
        /* name is optional, but may help identify the timer when debugging */
        .name = "anneal_timer_ui_update"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_ui_update, &anneal_timer_ui_update));

    // Timer to control the ZVS power (This should really rather be done manually in stead of a timer callback)
    const esp_timer_create_args_t periodic_timer_args_zvs_power = {
        .callback = &ame_anneal_timer_callback_zvs_power,
        .dispatch_method = ESP_TIMER_TASK,
        /* name is optional, but may help identify the timer when debugging */
        .name = "anneal_timer_zvs_power"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_zvs_power, &anneal_timer_zvs_power));

    // Use high precision timer for annealing
    const esp_timer_create_args_t anneal_timer_args = {
        .callback = &ame_anneal_timer_callback,
        .dispatch_method = ESP_TIMER_ISR,
        /* name is optional, but may help identify the timer when debugging */
        .name = "anneal_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&anneal_timer_args, &anneal_timer));

    PS_PUB_STR_FL("ame.gui.message", "IDLE", PS_FL_STICKY);

    for(;;)
    {
        if (xTaskNotifyWaitIndexed(EVENT_START_ANNEAL, 0, 0xFFFFFFFF, NULL, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Start");
            AME_COOLING_Start();
            annealSessionActive = true;
            PS_PUB_STR_FL("ame.gui.message", "STARTING", PS_FL_STICKY);
            // Disable ZVS FETS
            gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            // Enable ZVS main power
            ESP_ERROR_CHECK(esp_timer_start_periodic(anneal_timer_zvs_power, 2500));

            totalCases = AME_CFG_GetCasesCount();
            remainingCases = totalCases;
            casesDone = 0;
            PUB_INT_FL("ame.anneal.casesDone", casesDone, FL_STICKY);
            PUB_INT_FL("ame.anneal.casesRemain", remainingCases, FL_STICKY);

            ame_anneal_Run();

            // Disable ZVS main power
            if (esp_timer_is_active(anneal_timer_zvs_power))
            {
                ESP_ERROR_CHECK(esp_timer_stop(anneal_timer_zvs_power));
            }

            // Make sure this is off else 5W resistors gets hot
            gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 0);
        }
    }
}

/**
 * @brief Starts the annealing process by notifying the annealing task handle with the start event.
 *
 */
void AME_ANNEAL_Start(void)
{
    if (s_ame_anneal_task_handle != NULL)
    {
        xTaskNotifyIndexed(s_ame_anneal_task_handle, EVENT_START_ANNEAL, eNoAction, 0);
    }
}

void AME_ANNEAL_Stop(void)
{
    ESP_LOGI(TAG, "Stop");
    if (isAnnealing)
    {
        gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 1);
    }
    isAnnealing = false;
    annealSessionActive = false;
    
    // turn off SSR
    esp_timer_stop(anneal_timer);
    esp_timer_stop(anneal_timer_zvs_power);

    xTaskNotifyIndexed(s_ame_anneal_task_handle, EVENT_STOP_ANNEAL, 0, eIncrement);
}

bool AME_ANNEAL_IsAnnealing(void)
{
    return annealSessionActive;
}

void AME_ANNEAL_MainPowerState(bool state)
{
    if (state)
    {
        ESP_ERROR_CHECK(esp_timer_start_periodic(anneal_timer_zvs_power, 2500));
    }
    else
    {
        ESP_ERROR_CHECK(esp_timer_stop(anneal_timer_zvs_power));
    }
}

void AME_ANNEAL_ControlState(bool state)
{
    if (state)
    {
        gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 0);
    }
    else
    {
        gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 1);
    }
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/
static void ame_anneal_Run(void)
{
    xTaskNotifyStateClearIndexed(s_ame_anneal_task_handle, EVENT_STOP_ANNEAL);
    ps_subscriber_t *subTemp = ps_new_subscriber(1, STRLIST("ame.sensor.temp.1"));
    ps_msg_t *msg = NULL;
    float temperature = 0.0;
    int delayTime = 0;

    // Start case feed
    ESP_LOGI(TAG, "Fetch First Case");
    AME_STEPPER_FetchCase();

    for(;;)
    {
        annealTime_ms = AME_CFG_GetTime();

        // Wait for case ready
        while(AME_CASE_FEEDER_IsCaseReadyToDrop() == false)
        {
            if (xTaskNotifyWaitIndexed(EVENT_STOP_ANNEAL, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                goto stop_anneal;
            }
        }

        // Overheat timeout
        msg = ps_get(subTemp, 0);
        if (msg != NULL)
        {
            if (PS_IS_DBL(msg))
            {
                temperature = msg->dbl_val;
            }
        }
        ps_unref_msg(msg);

        if (temperature > CONFIG_TEMPERATURE_SLOW_DOWN)
        {
            // add 200ms Delay for every degree over the threshold
            delayTime = (CONFIG_TEMPERATURE_SLOW_DOWN - temperature) * 200u;
            ESP_LOGW(TAG, "Overheating, delay %d ms", delayTime);
            vTaskDelay(pdMS_TO_TICKS(delayTime));
        }

        // Drop case to shelf/coil
        ESP_LOGI(TAG, "Drop Case");
        PS_PUB_STR_FL("ame.gui.message", "WAIT CASE", PS_FL_STICKY);
        AME_STEPPER_DropCase();

        if (xTaskNotifyWaitIndexed(EVENT_STOP_ANNEAL, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            goto stop_anneal;
        }

        // Start annealing
        isAnnealing = true;
        ESP_LOGI(TAG, "Start annealing for %d ms", annealTime_ms);
        PS_PUB_STR_FL("ame.gui.message", "ANNEALING", PS_FL_STICKY);
        AME_ADC_StartMonitor();
        esp_timer_start_once(anneal_timer, annealTime_ms * 1000ull);
        gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 0);
        esp_timer_start_periodic(anneal_timer_ui_update, 50ull * 1000ull);

        casesDone++;
        remainingCases--;
        // Check if we are done with number of set cases, before we fetch next case
        if (casesDone < totalCases)
        {
            // Fetch Next Case
            ESP_LOGI(TAG, "Fetch Next Case");
            AME_STEPPER_FetchCase();
        }

        // Wait for annealing to finish
        while (isAnnealing)
        {
            if (xTaskNotifyWaitIndexed(EVENT_STOP_ANNEAL, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                AME_ADC_StopMonitor();
                goto stop_anneal;
            }
        }
        AME_ADC_StopMonitor();

        ESP_LOGI(TAG, "Drop Case");
        // Drop case
        PS_PUB_STR_FL("ame.gui.message", "DROP CASE", PS_FL_STICKY);
        
        if (casesDone < totalCases)
        {
            AME_DROPGATE_DropCase();
        }

        if (xTaskNotifyWaitIndexed(EVENT_STOP_ANNEAL, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            goto stop_anneal;
        }

        PUB_INT_FL("ame.anneal.casesDone", casesDone, FL_STICKY);
        PUB_INT_FL("ame.anneal.casesRemain", remainingCases, FL_STICKY);

        // Finish annealing
        if (casesDone >= totalCases)
        {
            ESP_LOGI(TAG, "All cases annealed");
            PS_PUB_STR_FL("ame.gui.message", "ALL DONE", PS_FL_STICKY);
            goto stop_anneal;
        }
    }

stop_anneal:
    ps_free_subscriber(subTemp);
    ame_anneal_End();
}

static void ame_anneal_End(void)
{
    gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 1);
    isAnnealing = false;
    annealSessionActive = false;
    // turn off SSR
    esp_timer_stop(anneal_timer);
    esp_timer_stop(anneal_timer_zvs_power);
    esp_timer_stop(anneal_timer_ui_update);

    AME_STEPPER_GoIdle();
    AME_DROPGATE_DropCase();
    totalCases = AME_CFG_GetCasesCount();
    annealTime_ms = AME_CFG_GetTime();

    remainingCases = AME_CFG_GetCasesCount() - totalCases;
    PUB_INT_FL("ame.anneal.casesDone", casesDone, FL_STICKY);
    PUB_INT_FL("ame.anneal.casesRemain", remainingCases, FL_STICKY);
    PUB_INT_FL("ame.anneal.time", annealTime_ms, FL_STICKY);
    PS_PUB_STR_FL("ame.gui.message", "IDLE", PS_FL_STICKY);

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "End");
}

static void ame_anneal_timer_callback(void *arg)
{
    esp_timer_stop(anneal_timer);
    gpio_set_level(CONFIG_IO_PIN_ZVS_CTRL, 1);
    isAnnealing = false;
}

static void ame_anneal_timer_callback_ui_update(void *arg)
{
    if (annealTime_ms > 50)
    {
        annealTime_ms -= 50;
    }
    else
    {
        annealTime_ms = 0;
    }

    if (annealTime_ms <= 0)
    {
        annealTime_ms = 0;
        esp_timer_stop(anneal_timer_ui_update);
    }
    PUB_INT_FL("ame.anneal.time", annealTime_ms, FL_STICKY);
}

static void ame_anneal_timer_callback_zvs_power(void *arg)
{
    static int pinState = 0;
    pinState = !pinState;
    gpio_set_level(CONFIG_IO_PIN_ZVS_ON_OFF, pinState);
}

/*************** END OF FUNCTIONS ************************************************************************************/
