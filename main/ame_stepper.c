 /*********************************************************************************************************************
 * Filename  : ame_stepper.c
 * Author    : ChristiaanAlberts
 * Created on: 26 March 2024
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_stepper.c
 * @brief  	Stepper motor control task.
 * Because some of the stepper driver control pins are shared both stepper motors are handled in the same task and
 * only one stepper motor can be controlled at a time.
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ame_stepper.h"
#include "ame_cfg.h"
#include "pubsub.h"

static const char *TAG = "ame_stepper";

/**********************************************************************************************************************
* Module Preprocessor Constants
**********************************************************************************************************************/
#define OUTPUT_MASK         (1ULL << CONFIG_IO_PIN_STEPPER_COMMON_DIR | 1ULL << CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST | 1ULL << CONFIG_IO_PIN_STEPPER_CASE_FEEDER_STEP | 1ULL << CONFIG_IO_PIN_STEPPER_CHAM_RST | 1ULL << CONFIG_IO_PIN_STEPPER_CHAM_STEP)
#define INPUT_MASK          (1ULL << CONFIG_IO_PIN_STEPPER_COMMON_FAULT)
#define HOME_SWITCH_MASK    (1ULL << CONFIG_IO_PIN_CHAM_HOME_SWITCH)

#define EVENT_PULSES_ZERO                   (0x01)
#define EVENT_CASE_FEEDER_FETCH_CASE        (0x02)
#define EVENT_CASE_FEEDER_DROP_CASE         (0x03)
#define EVENT_CASE_FEEDER_GO_IDLE           (0x04)
#define EVENT_CHAM_ADJUST_HEIGHT            (0x05)

/**********************************************************************************************************************
* Module Preprocessor Macros
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Typedefs
**********************************************************************************************************************/

/**********************************************************************************************************************
* Module Variable Definitions
**********************************************************************************************************************/
static int32_t steps = 0;
static esp_timer_handle_t periodic_timer_case_feeder;
static esp_timer_handle_t periodic_timer_cham;

static int cham_current_position_steps = 0;
static float cham_current_position_mm = 0.0f;

static const uint16_t feederSpeedTime[5] = {900, 750, 600, 450, 300};

static atomic_bool caseReady = false;


/**********************************************************************************************************************
* Global Variable Definitions
**********************************************************************************************************************/
extern TaskHandle_t s_ame_stepper_task_handle;

/**********************************************************************************************************************
* Module Function Prototypes
**********************************************************************************************************************/
static void periodic_timer_callback_cham(void *arg);
static void periodic_timer_callback_case_feeder(void *arg);

static bool ame_stepper_WaitEventOrGoIdle(int event);
static bool ame_stepper_WaitHomeSwitchOrPulsesZero(int homeSwitchState);

static void ame_stepper_CaseFeederMain(void);
static void ame_stepper_ChamMain(void);
static void ame_stepper_HomeCHAM(void);
static void ame_stepper_ChamSetDirDown(void);
static void ame_stepper_ChamSetDirUp(void);
static void ame_stepper_CaseFeederSetDirClockwise(void);
static void ame_stepper_CaseFeederSetDirCounterClockwise(void);

/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void AME_STEPPER_Task(void *pvArgs)
{
    const esp_timer_create_args_t periodic_timer_args_case_feeder = {
        .callback = &periodic_timer_callback_case_feeder,
        .dispatch_method = ESP_TIMER_TASK,
        /* name is optional, but may help identify the timer when debugging */
        .name = "step_timer_case_feeder"};

    const esp_timer_create_args_t periodic_timer_args_cham = {
        .callback = &periodic_timer_callback_cham,
        .dispatch_method = ESP_TIMER_TASK,
        /* name is optional, but may help identify the timer when debugging */
        .name = "step_timer_cham"};

    gpio_reset_pin(CONFIG_IO_PIN_CHAM_HOME_SWITCH);
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = OUTPUT_MASK;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = INPUT_MASK;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // set as input mode, interrupt any edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = HOME_SWITCH_MASK;
    // disable pull-up mode
    io_conf.pull_up_en = 1;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // install gpio isr service
    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // // hook isr handler for home switch gpio pin
    // gpio_isr_handler_add(CONFIG_IO_PIN_CHAM_HOME_SWITCH, gpio_isr_handler, (void *)CONFIG_IO_PIN_CHAM_HOME_SWITCH);

    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0);   // Disable case feeder stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 0);          // Disable CHAM stepper

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_case_feeder, &periodic_timer_case_feeder));
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_cham, &periodic_timer_cham));

    // Do homing sequence for CHAM
    ame_stepper_HomeCHAM();

    for (;;)
    {
        if (xTaskNotifyWaitIndexed(EVENT_CASE_FEEDER_FETCH_CASE, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ame_stepper_CaseFeederMain();
        }

        if (xTaskNotifyWaitIndexed(EVENT_CHAM_ADJUST_HEIGHT, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ame_stepper_ChamMain();
        }  
    }
}

/**
 * Notifies the stepper task to adjust the height of the CHAM.
 *
 * @return None.
 */
void AME_STEPPER_AdjustHeight(void)
{
    if (s_ame_stepper_task_handle != NULL)
    {
        xTaskNotifyIndexed(s_ame_stepper_task_handle, EVENT_CHAM_ADJUST_HEIGHT, eNoAction, 0);
    }
}

/**
 * Notifies the stepper task to fetch a case.
 *
 * @param None
 *
 * @return None
 */
void AME_STEPPER_FetchCase(void)
{
    if (s_ame_stepper_task_handle != NULL)
    {
        xTaskNotifyIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_FETCH_CASE, eNoAction, 0);
    }
}

/**
 * Check if the case is ready to be dropped.
 *
 * @return true if the case is ready to be dropped, false otherwise.
 */
bool AME_CASE_FEEDER_IsCaseReadyToDrop(void)
{
    return caseReady;
}

/**
 * Notifies the stepper task to drop a case.
 *
 * @param None
 *
 * @return None
 */
void AME_STEPPER_DropCase(void)
{
    if (s_ame_stepper_task_handle != NULL)
    {
        xTaskNotifyIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_DROP_CASE, eNoAction, 0);
    }
}

// function to notify stepper task to go idle
void AME_STEPPER_GoIdle(void)
{
    if (s_ame_stepper_task_handle != NULL)
    {
        xTaskNotifyIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_GO_IDLE, eNoAction, 0);
    }
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/
/**
 * @brief Main control for the case feeder
 * 
 */
static void ame_stepper_CaseFeederMain(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 1); // Enable case feeder stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 0);        // Disable CHAM stepper

    ame_stepper_CaseFeederSetDirClockwise();

    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_GO_IDLE);
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_PULSES_ZERO);

    for(;;)
    {
        ESP_LOGI(TAG, "CF: Start Fetch case");
        gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        steps = 2200 * 2;
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_case_feeder, 60));
        //  Wait for pulses to reach zero
        if (!ame_stepper_WaitEventOrGoIdle(EVENT_PULSES_ZERO))
        {
            break;
        }

        ESP_LOGI(TAG, "CF: Grab case. Speed: %d", AME_CFG_GetFeederSpeed());
        steps = 1900 * 2;
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_case_feeder, feederSpeedTime[AME_CFG_GetFeederSpeed() - 1]));
        // Wait for pulses to reach zero
        if (!ame_stepper_WaitEventOrGoIdle(EVENT_PULSES_ZERO))
        {
            break;
        }

        ESP_LOGI(TAG, "CF: Move to drop");
        steps = 1750 * 2;
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_case_feeder, 60));
        // Wait for pulses to reach zero
        if (!ame_stepper_WaitEventOrGoIdle(EVENT_PULSES_ZERO))
        {
            break;
        }

        ESP_LOGI(TAG, "CF: Case Ready to Drop");
        gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0); // Disable case feeder stepper, prevent driver from getting hot
        caseReady = true;
        // wait for drop command
        if (!ame_stepper_WaitEventOrGoIdle(EVENT_CASE_FEEDER_DROP_CASE))
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 1); // Re-Enable case feeder stepper
        caseReady = false;
        ESP_LOGI(TAG, "CF: Drop case");
        steps = 550 * 2;
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_case_feeder, 100));
        // Wait for pulses to reach zero
        ame_stepper_WaitEventOrGoIdle(EVENT_PULSES_ZERO);

        vTaskDelay(pdMS_TO_TICKS(500));

        break;
    }

    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0); // Disable case feeder stepper
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_GO_IDLE);
}

/**
 * @brief Main control for the CHAM
 * 
 */
static void ame_stepper_ChamMain(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0); // Disable case feeder stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 1);        // Enable CHAM stepper
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_GO_IDLE);

    float setHeight = AME_CFG_GetHeight() / 1000.0f;

    // Get direction to move
    if (cham_current_position_mm > setHeight)
    {
        // Lower than target, move down
        ame_stepper_ChamSetDirUp();
        // calculate number of pulses to move
        steps = (cham_current_position_mm - setHeight) * CONFIG_CHAM_PULSES_PER_MM * 2;
    }
    else if (cham_current_position_mm < setHeight)
    {
        // Higher than target, move down
        ame_stepper_ChamSetDirDown();
        // calculate number of pulses to move
        steps = (setHeight - cham_current_position_mm) * CONFIG_CHAM_PULSES_PER_MM * 2;
    }
    else
    {
        // Already at correct position
        goto height_done;
    }

    ESP_LOGI(TAG, "CHAM: Adjust to height: %f, steps: %" PRIu32, (double)(AME_CFG_GetHeight() / 1000.0), steps);
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_PULSES_ZERO);
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_cham, 100));
    // Wait for pulses to reach zero
    xTaskNotifyWaitIndexed(EVENT_PULSES_ZERO, 0, ULONG_MAX, NULL, portMAX_DELAY);

height_done:    
    ESP_LOGI(TAG, "CHAM: Height adjustment complete");
    cham_current_position_mm = setHeight;

    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0); // Disable case feeder stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 0);        // Disable CHAM stepper
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_CASE_FEEDER_GO_IDLE);
}

/**
 * @brief Initial homing of the stepper motor for the Case Height Adjustment Module.
 * 1. Move down fast until home switch is activated
 * 2. Move up slow until home switch is not activated.
 * 3. Move down slow until home switch is activated.
 * 
 */
static void ame_stepper_HomeCHAM(void)
{
    ESP_LOGI(TAG, "CHAM: Start Homing");
    // Enable CHAM Stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 1);

    // if home switch is not pressed, move down until we hit the home switch
    if (gpio_get_level(CONFIG_IO_PIN_CHAM_HOME_SWITCH) == 1)
    {
        // Home switch not yet activated
        goto fast_find_home;
    }
    else
    {
        // Home switch already activated
        goto home_activated;
    }

fast_find_home:
    ame_stepper_ChamSetDirDown();

    ESP_LOGI(TAG, "CHAM: Fast Home Switch Search");
    // Set to maximum number of steps the CHAM can move, will stop when home switch is activated
    steps = ((float)(CONFIG_CHAM_MAX_CASE_LENGTH - CONFIG_CHAM_MIN_CASE_LENGTH) / 1000.0f) * CONFIG_CHAM_PULSES_PER_MM * 2;
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_cham, 100));
    // Wait for pulses to reach zero
    ESP_LOGI(TAG, "CHAM: Wait home switch");
    if (!ame_stepper_WaitHomeSwitchOrPulsesZero(0))
    {
        ESP_LOGE(TAG, "CHAM: Home switch not found");
        goto end_home_search;
    }

home_activated:
    esp_timer_stop(periodic_timer_cham);
    
    ESP_LOGI(TAG, "CHAM: Home Switch Found");
    // move CHAM up until home switch no longer activated
    ame_stepper_ChamSetDirUp();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "CHAM: Backing off Home Switch");

    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_PULSES_ZERO);
    // Set to maximum number of steps the CHAM can move, will stop when home switch is activated
    steps = ((float)(CONFIG_CHAM_MAX_CASE_LENGTH - CONFIG_CHAM_MIN_CASE_LENGTH) / 1000.0f) * CONFIG_CHAM_PULSES_PER_MM * 2;
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_cham, 250));
    // Wait for pulses to reach zero
    if (!ame_stepper_WaitHomeSwitchOrPulsesZero(1))
    {
        ESP_LOGE(TAG, "CHAM: Home switch backoff fail - not found");
        goto end_home_search; 
    }

    esp_timer_stop(periodic_timer_cham);

    ESP_LOGI(TAG, "CHAM: Back off another 2000 pulses");

    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_PULSES_ZERO);
    // Set to maximum number of steps the CHAM can move, will stop when home switch is activated
    steps = 2000;
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_cham, 250));
    // Wait for pulses to reach zero
    xTaskNotifyWaitIndexed(EVENT_PULSES_ZERO, 0, 0xFFFFFFFF, NULL, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "CHAM: Home Switch Back off complete");

    // slow_find_home:
    ame_stepper_ChamSetDirDown();

    ESP_LOGI(TAG, "CHAM: Slow Home Switch Search");
    // Make sure home switch event is clear
    xTaskNotifyStateClearIndexed(s_ame_stepper_task_handle, EVENT_PULSES_ZERO);
    // Set to maximum number of steps the CHAM can move, will stop when home switch is activated
    steps = ((float)(CONFIG_CHAM_MAX_CASE_LENGTH - CONFIG_CHAM_MIN_CASE_LENGTH) / 1000.0f) * CONFIG_CHAM_PULSES_PER_MM * 2;
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_cham, 600));
    // Wait for pulses to reach zero
    if (!ame_stepper_WaitHomeSwitchOrPulsesZero(0))
    {
        ESP_LOGE(TAG, "CHAM: Home switch slow not found");
        goto end_home_search;
    }

    esp_timer_stop(periodic_timer_cham);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "CHAM: Homing Complete");
    cham_current_position_steps = 0;
    cham_current_position_mm = CONFIG_CHAM_MAX_CASE_LENGTH / 1000.0f;
    ESP_LOGI(TAG, "CHAM: Current Position: %f", CONFIG_CHAM_MAX_CASE_LENGTH / 1000.0f);

    ESP_LOGI(TAG, "CHAM: Adjust to last saved height: %f", AME_CFG_GetHeight() / 1000.0f);
    AME_STEPPER_AdjustHeight();

end_home_search:
    ESP_LOGI(TAG, "CHAM: End Homing");
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_RST, 0); // Disable case feeder stepper
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_RST, 0);        // Disable CHAM stepper
}

static void ame_stepper_ChamSetDirDown(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_COMMON_DIR, ((CONFIG_STEPPER_DIR_FOR_CHAM_UP == 1) ? 0 : 1));
}

static void ame_stepper_ChamSetDirUp(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_COMMON_DIR, ((CONFIG_STEPPER_DIR_FOR_CHAM_UP == 1) ? 1 : 0));
}

static void ame_stepper_CaseFeederSetDirClockwise(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_COMMON_DIR, ((CONFIG_STEPPER_DIR_FOR_CASE_FEEDER_CLOCKWISE == 1) ? 1 : 0));
}

static void ame_stepper_CaseFeederSetDirCounterClockwise(void)
{
    gpio_set_level(CONFIG_IO_PIN_STEPPER_COMMON_DIR, ((CONFIG_STEPPER_DIR_FOR_CASE_FEEDER_CLOCKWISE == 1) ? 0 : 1));
}

/**
 * @brief Wait for a specific event or the Go Idle command
 *
 * @param event The event to wait for
 * @return true If the event was received
 * @return false If the Go Idle command was received
 */
static bool ame_stepper_WaitEventOrGoIdle(int event)
{
    bool eventReceived = false;
    for(;;)
    {
        if (xTaskNotifyWaitIndexed(event, 0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(10)) == pdFALSE)
        {
            // Don't clear go idle event here to make sure we don't miss it, we will clear it on exit or before restart
            if (xTaskNotifyWaitIndexed(EVENT_CASE_FEEDER_GO_IDLE, 0, 0x0, NULL, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                break;
            }
        }
        else
        {
            eventReceived = true;
            break;
        }
    }
    return eventReceived;
}

static bool ame_stepper_WaitHomeSwitchOrPulsesZero(int homeSwitchState)
{
    bool homeSwitchEventReceived = false;
    for (;;)
    {
        if (gpio_get_level(CONFIG_IO_PIN_CHAM_HOME_SWITCH) != homeSwitchState)
        {
            if (xTaskNotifyWaitIndexed(EVENT_PULSES_ZERO, 0, 0xFFFFFFFF, NULL, 0) == pdTRUE)
            {
                break;
            }
        }
        else
        {
            homeSwitchEventReceived = true;
            break;
        }
    }
    return homeSwitchEventReceived;
}

static void periodic_timer_callback_case_feeder(void *arg)
{
    static uint8_t outputState = 0;

    outputState = !outputState;
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CASE_FEEDER_STEP, outputState);
    if(--steps <= 0)
    {
        esp_timer_stop(periodic_timer_case_feeder);
        BaseType_t xHigherPriorityTaskWoken;
        uint32_t ulStatusRegister;

        /* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
        xTaskNotifyFromISR() unblocks the handling task, and the priority of
        the handling task is higher than the priority of the currently running task,
        then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
        xHigherPriorityTaskWoken = pdFALSE;

        /* Unblock the handling task so the task can perform any processing necessitated
        by the interrupt.  xHandlingTask is the task's handle, which was obtained
        when the task was created.  The handling task's 0th notification value
        is bitwise ORed with the interrupt status - ensuring bits that are already
        set are not overwritten. */
        xTaskNotifyIndexedFromISR(s_ame_stepper_task_handle,
                                  EVENT_PULSES_ZERO,
                                  0,
                                  eIncrement,
                                  &xHigherPriorityTaskWoken);

        /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
        The macro used to do this is dependent on the port and may be called
        portEND_SWITCHING_ISR. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void periodic_timer_callback_cham(void *arg)
{
    static uint8_t outputState = 0;

    outputState = !outputState;
    gpio_set_level(CONFIG_IO_PIN_STEPPER_CHAM_STEP, outputState);
    // TODO: Add check to prevent it from going further down if already touching the home switch
    if (--steps <= 0)
    {
        esp_timer_stop(periodic_timer_cham);
        BaseType_t xHigherPriorityTaskWoken;

        /* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
        xTaskNotifyFromISR() unblocks the handling task, and the priority of
        the handling task is higher than the priority of the currently running task,
        then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
        xHigherPriorityTaskWoken = pdFALSE;

        /* Unblock the handling task so the task can perform any processing necessitated
        by the interrupt.  xHandlingTask is the task's handle, which was obtained
        when the task was created.  The handling task's 0th notification value
        is bitwise ORed with the interrupt status - ensuring bits that are already
        set are not overwritten. */
        xTaskNotifyIndexedFromISR(s_ame_stepper_task_handle,
                                  EVENT_PULSES_ZERO,
                                  0,
                                  eIncrement,
                                  &xHigherPriorityTaskWoken);

        /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
        The macro used to do this is dependent on the port and may be called
        portEND_SWITCHING_ISR. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*************** END OF FUNCTIONS ************************************************************************************/
