/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "cmd_system.h"
#include "cmd_nvs.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_app_desc.h"

#include "driver/gpio.h"

#include "ame_temperature.h"
#include "ame_ui.h"
#include "ame_cfg.h"
#include "ame_adc.h"
#include "ame_stepper.h"
#include "ame_cooling.h"
#include "ame_anneal.h"
#include "ame_buzzer.h"
#include "cmd_pubsub.h"
#include "cmd_control.h"
#include "ota_task.h"

#include "pubsub.h"
#include "ame_dropgate.h"

// #include "ds18b20_gpio.h"

static const char *TAG = "main";

TaskHandle_t s_display_task_handle = NULL;
TaskHandle_t s_ame_stepper_task_handle = NULL;
TaskHandle_t s_ame_cooling_task_handle = NULL;
TaskHandle_t s_ame_anneal_task_handle = NULL;

void app_main(void)
{

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG,"AME v2! App version: %s\n", app_desc->version);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    fflush(stdout);

    AME_CFG_Init();

    ps_init();

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = "AME>";

    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    register_system_common();
    register_nvs();
    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    CMD_PUBSUB_Init();
    CMD_CONTROL_Init();

    AME_DROPGATE_Init();

    xTaskCreate(AME_BUZZER_Task, "ame_buzzer", 1024 * 4, NULL, 1, NULL);
    xTaskCreate(AME_UI_Task, "ame_ui", 1024 * 12, NULL, 9, &s_display_task_handle);
    xTaskCreate(AME_TEMPERATURE_Task, "ame_temperature", 1024 * 4, NULL, 3, NULL);
    xTaskCreate(AME_ADC_Task, "ame_adc", 1024 * 8, NULL, 5, NULL);
    xTaskCreate(AME_STEPPER_Task, "ame_stepper", 1024 * 4, NULL, 6, &s_ame_stepper_task_handle);
    xTaskCreate(AME_COOLING_Task, "ame_cooling", 1024 * 4, NULL, 5, &s_ame_cooling_task_handle);
    xTaskCreate(AME_ANNEAL_Task, "ame_anneal", 1024 * 4, NULL, 10, &s_ame_anneal_task_handle);

    OTA_TASK_Init();

    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
