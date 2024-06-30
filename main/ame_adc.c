 /*********************************************************************************************************************
 * Filename  : current.c
 * Author    : ChristiaanAlberts
 * Created on: 28 September 2022
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	current.c
 * @brief  	Current measurement task
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
// #include "esp_adc/adc_monitor.h"

#include "ame_anneal.h"
#include "pubsub.h"

static const char *TAG = "ame_adc";

/**********************************************************************************************************************
 * Module Preprocessor Constants
 **********************************************************************************************************************/
#define READ_LEN                        256
#define ACS_SENSITIVITY                 44      // mV / A
#define ACS_ZERO_CURRENT_VOLTAGE_MV     (1650)
#define ADC_EXTERNAL_DIVIDER            (1.2/48.2)

#define AME_ADC_ATTEN_DB                ADC_ATTEN_DB_6
#define AME_ADC_BITWIDTH                SOC_ADC_DIGI_MAX_BITWIDTH

#define AME_ADC_CURRENT_TRIP_THRES_VOLT (uint32_t)(ACS_ZERO_CURRENT_VOLTAGE_MV - (((float)CONFIG_OVERCURRENT_THRESHOLD/1000.0f)*ACS_SENSITIVITY))       

/**********************************************************************************************************************
 * Module Preprocessor Macros
 **********************************************************************************************************************/
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)

/**********************************************************************************************************************
 * Module Typedefs
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Variable Definitions
 **********************************************************************************************************************/
static int rawVoltage[2];
static float vin_voltage = 0.0;
static float zvs_current = 0.0;

static TaskHandle_t s_task_handle;

// Channel 8 is Current, channel 9 is Voltage
static adc_channel_t channel[2] = {ADC_CHANNEL_8, ADC_CHANNEL_9};

static adc_cali_handle_t adc1_cali_chan8_handle = NULL;
static adc_cali_handle_t adc1_cali_chan9_handle = NULL;

// static adc_monitor_handle_t monitor_handle = NULL;
static atomic_bool enableMonitor = false;

static SemaphoreHandle_t adcMonitorSemaphore;

/**********************************************************************************************************************
 * Global Variable Definitions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Function Prototypes
 **********************************************************************************************************************/
static void ame_adc_continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
static bool ame_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void ame_adc_calibration_deinit(adc_cali_handle_t handle);
static uint16_t ame_adc_GetMonitorValue(void);
// static bool overcurrent_cb(adc_monitor_handle_t monitor_handle, const adc_monitor_evt_data_t *event_data, void *user_data);

/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void AME_ADC_Task(void *pvArgs)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[READ_LEN] = {0};
    memset(result, 0xcc, READ_LEN);
    static uint32_t currPubCounter = 0;
    static uint32_t voltPubCounter = 0;

    s_task_handle = xTaskGetCurrentTaskHandle();

    /* Create a mutex type semaphore. */
    adcMonitorSemaphore = xSemaphoreCreateMutex();

    vTaskDelay(pdMS_TO_TICKS(5000));

    adc_continuous_handle_t handle = NULL;
    ame_adc_continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    //-------------ADC1 Calibration Init---------------//
    ame_adc_calibration_init(ADC_UNIT_1, channel[0], AME_ADC_ATTEN_DB, &adc1_cali_chan8_handle);
    ame_adc_calibration_init(ADC_UNIT_1, channel[1], AME_ADC_ATTEN_DB, &adc1_cali_chan9_handle);

    adc_iir_filter_handle_t filter_handle = NULL;
    adc_continuous_iir_filter_config_t filter_config = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_8,
        .coeff = ADC_DIGI_IIR_FILTER_COEFF_64
    };
    ESP_ERROR_CHECK(adc_new_continuous_iir_filter(handle, &filter_config, &filter_handle));
    adc_continuous_iir_filter_enable(filter_handle);

    // Add filter on voltage channel
    adc_iir_filter_handle_t voltage_filter_handle = NULL;
    adc_continuous_iir_filter_config_t voltage_filter_config = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_9,
        .coeff = ADC_DIGI_IIR_FILTER_COEFF_64};
    ESP_ERROR_CHECK(adc_new_continuous_iir_filter(handle, &voltage_filter_config, &voltage_filter_handle));
    adc_continuous_iir_filter_enable(voltage_filter_handle);

    int monitor_value = ame_adc_GetMonitorValue();
    // adc_monitor_config_t  monitor_config = {
    //     .adc_unit = ADC_UNIT_1,
    //     .channel = ADC_CHANNEL_8,
    //     .l_threshold = monitor_value,
    //     .h_threshold = -1,
    // };
    // ESP_LOGI(TAG, "monitor_value = %"PRIi32, monitor_config.l_threshold);
    // adc_monitor_evt_cbs_t monitor_cb = {
    //     .on_over_high_thresh = NULL,
    //     .on_below_low_thresh = overcurrent_cb,
    // };

    // ESP_ERROR_CHECK(adc_new_continuous_monitor(handle, &monitor_config, &monitor_handle));
    // ESP_ERROR_CHECK(adc_continuous_monitor_register_event_callbacks(monitor_handle, &monitor_cb, NULL));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    
    while (1)
    {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
            if (ret == ESP_OK)
            {
                //ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
                // ESP_LOGI(TAG, "0");
                static uint16_t overCurrentCounter = 0;
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    uint32_t chan_num = ADC_GET_CHANNEL(p);
                    uint32_t data = ADC_GET_DATA(p);
                    if (chan_num == 8)
                    {
                        if(enableMonitor && (data < monitor_value))
                        {
                            if (++overCurrentCounter > 100)
                            {
                                ESP_LOGI(TAG, "zvs_current raw cutoff = %" PRIu32 " %d", data, rawVoltage[1]);
                                if (AME_ANNEAL_IsAnnealing())
                                {
                                    AME_ANNEAL_Stop();
                                }
                                overCurrentCounter = 0;
                            }
                        }
                        else
                        {
                            overCurrentCounter = 0;
                        }
                        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan8_handle, data, &rawVoltage[0]));
                        //rawVoltage[0] += 15;
                        zvs_current = (float)(ACS_ZERO_CURRENT_VOLTAGE_MV - rawVoltage[0]) / ACS_SENSITIVITY;
                        if (currPubCounter++ % 250 == 0)
                        {
                            PUB_DBL_FL("ame.sensor.current", zvs_current, FL_STICKY);
                            //ESP_LOGI(TAG, "zvs_current raw = %"PRIu32, data);
                        }
                    }
                    if (chan_num == 9)
                    {
                        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan9_handle, data, &rawVoltage[1]));
                        //rawVoltage[1] += 15;
                        vin_voltage = (float)rawVoltage[1] / ADC_EXTERNAL_DIVIDER;
                        if (voltPubCounter++ % 500 == 0)
                        {
                            PUB_DBL_FL("ame.sensor.voltage", vin_voltage, FL_STICKY);
                        }
                    }
                }
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ame_adc_calibration_deinit(adc1_cali_chan8_handle);
    ame_adc_calibration_deinit(adc1_cali_chan9_handle);
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

void AME_ADC_StartMonitor(void)
{
    if (adcMonitorSemaphore != NULL)
    {
        /* See if we can obtain the semaphore.  If the semaphore is not
        available wait 10 ticks to see if it becomes free. */
        if (xSemaphoreTake(adcMonitorSemaphore, (TickType_t)10) == pdTRUE)
        {
            /* We were able to obtain the semaphore and can now access the
            shared resource. */

            enableMonitor = true;
            //ESP_ERROR_CHECK(adc_continuous_monitor_enable(monitor_handle));

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive(adcMonitorSemaphore);
        }
        else
        {
            /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
        }
    }
}

void AME_ADC_StopMonitor(void)
{
    if (adcMonitorSemaphore != NULL)
    {
        /* See if we can obtain the semaphore.  If the semaphore is not
        available wait 10 ticks to see if it becomes free. */
        if (xSemaphoreTake(adcMonitorSemaphore, (TickType_t)10) == pdTRUE)
        {
            /* We were able to obtain the semaphore and can now access the
            shared resource. */

            enableMonitor = false;
            // ESP_ERROR_CHECK(adc_continuous_monitor_disable(monitor_handle));

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive(adcMonitorSemaphore);
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
static void ame_adc_continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = AME_ADC_ATTEN_DB;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = AME_ADC_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool ame_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void ame_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static uint16_t ame_adc_GetMonitorValue(void)
{
    int guess = pow(2, SOC_ADC_DIGI_MAX_BITWIDTH) / 2;
    int guessStep = guess / 2;
    int calculatedVoltage = 0;
    int i;

    for (i = 0; ((i < 32) && (guessStep > 0)); i++)
    {
        adc_cali_raw_to_voltage(adc1_cali_chan8_handle, guess, &calculatedVoltage);
        if (calculatedVoltage < AME_ADC_CURRENT_TRIP_THRES_VOLT)
        {
            guess += guessStep;
        }
        else if (calculatedVoltage > AME_ADC_CURRENT_TRIP_THRES_VOLT)
        {
            guess -= guessStep;
        }
        else
        {
            break;
        }
        guessStep /= 2;
    }

    ESP_LOGI(TAG, "guess: %d:%d:%d", guess, i, calculatedVoltage);

    return guess;
}

// static bool IRAM_ATTR overcurrent_cb(adc_monitor_handle_t monitor_handle, const adc_monitor_evt_data_t *event_data, void *user_data)
// {
//     AME_ANNEAL_Stop();
//     return true;
// }

/*************** END OF FUNCTIONS ************************************************************************************/
