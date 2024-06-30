/*********************************************************************************************************************
 * Author    : Christiaan Alberts
 * Created on: Mar 19 2024
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_temperature.c
 * @brief  	Temperature sensor handling
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "pubsub.h"

static const char *TAG = "temperature";

#define MAX_DEVICES        (2)
#define DS18B20_RESOLUTION (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD      (1000) // milliseconds

#define ESP_PRINT_MAC_FMT   "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_PRINT_MAC_ARG(mac) (mac)[0], (mac)[1], (mac)[2], (mac)[3], (mac)[4], (mac)[5]

/**********************************************************************************************************************
 * Module Preprocessor Constants
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Preprocessor Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Typedefs
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Module Variable Definitions
 **********************************************************************************************************************/
OneWireBus_ROMCode rom_codeTemp;


/**********************************************************************************************************************
 * Global Variable Definitions
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Module Function Prototypes
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void AME_TEMPERATURE_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    gpio_reset_pin(CONFIG_IO_PIN_DS18B20);
    esp_rom_gpio_pad_select_gpio(CONFIG_IO_PIN_DS18B20);

    // Stable readings require a brief period before communication
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Initialize 1-Wire bus
    OneWireBus *owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, CONFIG_IO_PIN_DS18B20, RMT_CHANNEL_2, RMT_CHANNEL_5);
    owb_use_crc(owb, true); // enable CRC check for ROM code

    // Find connected devices
    /**** Inlet DS18B20 ****/
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    while(!found)
    {
        owb_search_first(owb, &search_state, &found);
        vTaskDelay(1000.0 / portTICK_PERIOD_MS);
    }
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "%d : %s\n", num_devices, rom_code_s);
        rom_codeTemp = search_state.rom_code;
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        if (num_devices >= MAX_DEVICES)
        {
            break;
        }
        owb_search_next(owb, &search_state, &found);
    }
    // else
    // {
    //     ESP_LOGE(TAG, "Temp Sensor Init Fail");
    // }
    ESP_LOGI(TAG, "Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info *devices[MAX_DEVICES] = {0};
    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info *ds18b20_info = ds18b20_malloc(); // heap allocation
        devices[i] = ds18b20_info;

        // if (num_devices == 1)
        // {
        //     ESP_LOGI(TAG, "Single device optimisations enabled\n");
        //     ds18b20_init_solo(ds18b20_info, owb); // only one device on bus
        // }
        // else
        {
            ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(ds18b20_info, true); // enable CRC check on all reads
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }

    esp_log_level_set("ds18b20", ESP_LOG_NONE);
    esp_log_level_set("owb_rmt", ESP_LOG_NONE);

    for (;;)
    {
        /**** Read Inlet Temperature ****/
        ds18b20_convert_all(owb);
        // In this application all devices use the same resolution,
        // so use the first device to determine the delay
        ds18b20_wait_for_conversion(devices[0]);

        // Read the results immediately after conversion otherwise it may fail
        // (using printf before reading may take too long)
        float readings[MAX_DEVICES] = {0};
        DS18B20_ERROR errors[MAX_DEVICES] = {0};
        for (int i = 0; i < num_devices; i++)
        {
            errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
            //ESP_LOGI(TAG, ESP_PRINT_MAC_FMT " - %f", ESP_PRINT_MAC_ARG(devices[i]->rom_code.fields.serial_number), readings[i]);
        }

        // Publish temperature
        PUB_DBL_FL("ame.sensor.temp.1", readings[0], FL_STICKY);
        PUB_DBL_FL("ame.sensor.temp.2", readings[1], FL_STICKY);

        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

/*************** END OF FUNCTIONS ************************************************************************************/
