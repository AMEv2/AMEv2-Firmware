 /*********************************************************************************************************************
 * Filename  : ota_task.h
 * Author    : ChristiaanAlberts
 * Created on: 27 May 2024
  ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ota_task.h
 * @brief  	OTA task
 *
 *********************************************************************************************************************/
#ifndef OTA_TASK_H_
#define OTA_TASK_H_

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "esp_netif.h"

#ifdef __cplusplus
extern “C” {
#endif



/**********************************************************************************************************************
* Preprocessor Constants
**********************************************************************************************************************/


/**********************************************************************************************************************
* Global Configuration Constants
**********************************************************************************************************************/


/**********************************************************************************************************************
* Global Macros
**********************************************************************************************************************/

	
/**********************************************************************************************************************
* Global Typedefs
**********************************************************************************************************************/
typedef struct
{
    char ssid[16];
    char password[9];
    esp_ip4_addr_t ip;
} ota_task_wifi_info_t;

/**********************************************************************************************************************
* Global Variables
**********************************************************************************************************************/


/**********************************************************************************************************************
* Global Function Prototypes
**********************************************************************************************************************/
void OTA_TASK_Init(void);

void OTA_TASK_Start(void);

bool OTA_TASK_IsStarted(void);

void OTA_TASK_GetWifiInfo(ota_task_wifi_info_t * wifi_info);

#ifdef __cplusplus
}
#endif

#endif /* OTA_TASK_H_ */

/*** End of File *****************************************************************************************************/
