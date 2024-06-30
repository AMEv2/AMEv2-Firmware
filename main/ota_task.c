 /*********************************************************************************************************************
 * Filename  : ota_task.c
 * Author    : ChristiaanAlberts
 * Created on: 27 May 2024
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ota_task.c
 * @brief  	OTA Task
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_random.h"

#include "ota_task.h"

#include "ame_cfg.h"

static const char *TAG = "ota_task";

/**********************************************************************************************************************
 * Module Preprocessor Constants
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Module Preprocessor Macros
 **********************************************************************************************************************/
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/**********************************************************************************************************************
 * Module Typedefs
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Variable Definitions
 **********************************************************************************************************************/
static esp_netif_t *netif = NULL;
static bool started = false;

/**********************************************************************************************************************
 * Global Variable Definitions
 **********************************************************************************************************************/
/*
 * Serve OTA update portal (index.html)
 */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");

/**********************************************************************************************************************
 * Module Function Prototypes
 **********************************************************************************************************************/
static esp_err_t softap_init(void);
static esp_err_t http_server_init(void);
static esp_err_t update_post_handler(httpd_req_t *req);
static esp_err_t index_get_handler(httpd_req_t *req);
static esp_err_t favicon_ico_handler(httpd_req_t *req);

/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void OTA_TASK_Init(void)
{
    /* Mark current app as valid */
    const esp_partition_t *partition = esp_ota_get_running_partition();
    printf("Currently running partition: %s\r\n", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

void OTA_TASK_Start(void)
{
    if(!started)
    {
        started = true;
        ESP_LOGI(TAG, "Enabling WiFi OTA...");
        ESP_ERROR_CHECK(softap_init());
        ESP_ERROR_CHECK(http_server_init());
    }
}

bool OTA_TASK_IsStarted(void)
{
    return started;
}

void OTA_TASK_GetWifiInfo(ota_task_wifi_info_t *wifi_info)
{
    esp_netif_ip_info_t ip_info;
    AME_CFG_GetWifiPassword(wifi_info->password, sizeof(wifi_info->password));
    strcpy(wifi_info->ssid, "AMEv2");
    esp_netif_get_ip_info(netif, &ip_info);
    wifi_info->ip = ip_info.ip;
}

/**********************************************************************************************************************
* Module Function Definitions
**********************************************************************************************************************/
static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t favicon_ico_handler(httpd_req_t *req)
{
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_end - favicon_ico_start);
    return ESP_OK;
}

/*
 * Handle OTA file upload
 */
static esp_err_t update_post_handler(httpd_req_t *req)
{
    char buf[1000];
    esp_ota_handle_t ota_handle;
    int remaining = req->content_len;

    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

    while (remaining > 0)
    {
        int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

        // Timeout Error: Just retry
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;

            // Serious Error: Abort OTA
        }
        else if (recv_len <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
            return ESP_FAIL;
        }

        // Successful Upload: Flash firmware chunk
        if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
            return ESP_FAIL;
        }

        remaining -= recv_len;
    }

    // Validate and switch to new OTA image and reboot
    if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");

    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_restart();

    return ESP_OK;
}

/*
 * HTTP Server
 */
static httpd_uri_t index_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_get_handler,
    .user_ctx = NULL};

static httpd_uri_t update_post = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_post_handler,
    .user_ctx = NULL};

static httpd_uri_t favicon_ico = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = favicon_ico_handler,
    .user_ctx = NULL};

static esp_err_t http_server_init(void)
{
    static httpd_handle_t http_server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    if (httpd_start(&http_server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(http_server, &index_get);
        httpd_register_uri_handler(http_server, &update_post);
        httpd_register_uri_handler(http_server, &favicon_ico);
    }

    return http_server == NULL ? ESP_FAIL : ESP_OK;
}

/*
 * WiFi configuration
 */
static esp_err_t softap_init(void)
{
    esp_err_t res = ESP_OK;
    char wifiPwd[9] = {0};

    res |= esp_netif_init();
    res |= esp_event_loop_create_default();
    netif = esp_netif_create_default_wifi_ap();

    if (AME_CFG_GetWifiPassword(wifiPwd, sizeof(wifiPwd)) != ESP_OK)
    {
        // password empty, create random 8 digit password
        for (int i = 0; i < 8; i++)
        {
            wifiPwd[i] = esp_random() % 10 + '0';
        }
        // make sure first digit is not '0'
        if (wifiPwd[0] == '0')
        {
            wifiPwd[0] = esp_random() % 9 + '1';
        }
        AME_CFG_SetWifiPassword(wifiPwd, sizeof(wifiPwd));
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    res |= esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "AMEv2",
            .ssid_len = strlen("AMEv2"),
            .channel = 6,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .password = "12345678",
            .max_connection = 3
            },
    };

    memcpy(wifi_config.ap.password, wifiPwd, strlen(wifiPwd));

    ESP_LOGI(TAG, "Starting AP: %s, password: %s", wifi_config.ap.ssid, wifi_config.ap.password);

    res |= esp_wifi_set_mode(WIFI_MODE_AP);
    res |= esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    res |= esp_wifi_start();

    return res;
}

/*************** END OF FUNCTIONS ************************************************************************************/
