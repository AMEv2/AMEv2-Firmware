 /*********************************************************************************************************************
 * Filename  : ame_cfg.c
 * Author    : ChristiaanAlberts
 * Created on: 27 September 2022
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_cfg.c
 * @brief  	config handling
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "ame_cfg";

/**********************************************************************************************************************
* Module Preprocessor Constants
**********************************************************************************************************************/
#define AME_CFG_SERIAL_NUMBER       "SerialNumber"
#define AME_CFG_TIME                "time"
#define AME_CFG_CASECOUNT           "caseCount"
#define AME_CFG_HEIGHT              "Height"
#define AME_CFG_FEED_SPEED          "FeedSpeed"
#define AME_CFG_STARTUP_TUNE        "StartTune"
#define AME_CFG_STARTUP_TUNE_VOL    "StartTuneVol"
#define AME_CFG_WIFI_PASSWORD       "WifiPassword"

/**********************************************************************************************************************
* Module Preprocessor Macros
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Typedefs
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Variable Definitions
**********************************************************************************************************************/
static nvs_handle_t nvsHandle;

/**********************************************************************************************************************
* Global Variable Definitions
**********************************************************************************************************************/


/**********************************************************************************************************************
* Module Function Prototypes
**********************************************************************************************************************/


/**********************************************************************************************************************
* Global Function Definitions
**********************************************************************************************************************/
void AME_CFG_Init(void)
{
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_open("ame_config", NVS_READWRITE, &nvsHandle);
}

void AME_CFG_SetTime(uint16_t time_ms)
{
    nvs_set_u16(nvsHandle, AME_CFG_TIME, time_ms);
}

uint16_t AME_CFG_GetTime(void)
{
    uint16_t time;
    if (nvs_get_u16(nvsHandle, AME_CFG_TIME, &time) != ESP_OK)
    {
        time = 2100;
    }
    return time;
}

/**
 * @brief Set the height in micrometers
 * 
 * @param height_um 
 */
void AME_CFG_SetHeight(uint32_t height_um)
{
    nvs_set_u32(nvsHandle, AME_CFG_HEIGHT, height_um);
}

/**
 * @brief Get the height in micrometers
 * 
 * @return uint32_t 
 */
uint32_t AME_CFG_GetHeight(void)
{
    uint32_t height_um;
    if (nvs_get_u32(nvsHandle, AME_CFG_HEIGHT, &height_um) != ESP_OK)
    {
        height_um = 40000;
    }
    return height_um;
}

void AME_CFG_SetCasesCount(int16_t caseCount)
{
    nvs_set_i16(nvsHandle, AME_CFG_CASECOUNT, caseCount);
}

int16_t AME_CFG_GetCasesCount(void)
{
    int16_t caseCount;
    if (nvs_get_i16(nvsHandle, AME_CFG_CASECOUNT, &caseCount) != ESP_OK) 
    {
        caseCount = 50;
    }
    return caseCount;
}

void AME_CFG_SetFeederSpeed(int8_t feedSpeed)
{
    nvs_set_i8(nvsHandle, AME_CFG_FEED_SPEED, feedSpeed);
}

int8_t AME_CFG_GetFeederSpeed(void)
{
    int8_t feedSpeed;
    if (nvs_get_i8(nvsHandle, AME_CFG_FEED_SPEED, &feedSpeed) != ESP_OK)
    {
        feedSpeed = 5;
    }
    return feedSpeed;
}

void AME_CFG_SetStartupTune(int8_t tune)
{
    nvs_set_i8(nvsHandle, AME_CFG_STARTUP_TUNE, tune);
}

int8_t AME_CFG_GetStartupTune(void)
{
    int8_t tune;
    if (nvs_get_i8(nvsHandle, AME_CFG_STARTUP_TUNE, &tune) != ESP_OK)
    {
        tune = 1;
    }
    return tune;
}

void AME_CFG_SetStartupTuneVol(int8_t vol)
{
    nvs_set_i8(nvsHandle, AME_CFG_STARTUP_TUNE_VOL, vol);
}

/**
 * Retrieves the startup tune volume from non-volatile storage.
 *
 * @return The startup tune volume as an integer between 0 and 255.
 */
int8_t AME_CFG_GetStartupTuneVol(void)
{
    int8_t vol;
    if (nvs_get_i8(nvsHandle, AME_CFG_STARTUP_TUNE_VOL, &vol) != ESP_OK)
    {
        vol = 6;
    }
    return vol;
}

esp_err_t AME_CFG_GetWifiPassword(char *buf, size_t bufSize)
{
    if (nvs_get_blob(nvsHandle, AME_CFG_WIFI_PASSWORD, buf, &bufSize) != ESP_OK)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void AME_CFG_SetWifiPassword(char *buf, size_t bufSize)
{
    nvs_set_blob(nvsHandle, AME_CFG_WIFI_PASSWORD, buf, bufSize);
}

uint32_t AME_CFG_GetSerialNumber(void)
{
    uint32_t serialNumber;
    if (nvs_get_u32(nvsHandle, AME_CFG_SERIAL_NUMBER, &serialNumber) != ESP_OK)
    {
        serialNumber = 0;
    }
    
    return serialNumber;
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/


/*************** END OF FUNCTIONS ************************************************************************************/
