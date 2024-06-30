/*********************************************************************************************************************
 * Author    : Christiaan Alberts
 * Created on: Mar 19 2024
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	ame_temperature.h
 * @brief  	Temperature sensor hanldling
 *
 *********************************************************************************************************************/
#ifndef AME_TEMPERATURE_H_
#define AME_TEMPERATURE_H_

/**********************************************************************************************************************
* Includes
**********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


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


/**********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/


/**********************************************************************************************************************
* Global Function Prototypes
**********************************************************************************************************************/
void AME_TEMPERATURE_Task(void * pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* AME_TEMPERATURE_H_ */

/*** End of File *****************************************************************************************************/
