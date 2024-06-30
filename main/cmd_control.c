/*********************************************************************************************************************
 * Author    : Christiaan Alberts
 * Created on: 23 June 2024
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	cmd_anneal.c
 * @brief  	annealer console commands
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ame_anneal.h"
#include "cmd_control.h"

static const char *TAG = "cmd_control";

/**********************************************************************************************************************
 * Module Preprocessor Constants
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Preprocessor Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Typedefs
 **********************************************************************************************************************/
/** Arguments used by 'control' function */
static struct
{
    struct arg_str *unit;
    struct arg_str *state;
    struct arg_end *end;
} control_args;



/**********************************************************************************************************************
 * Module Variable Definitions
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Global Variable Definitions
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Module Function Prototypes
 **********************************************************************************************************************/
static int cmd_control_control_console(int argc, char **argv);
static void cmd_control_register_console(void);


/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void CMD_CONTROL_Init(void)
{
    ESP_LOGI(TAG, "CMD Anneal Init");
    cmd_control_register_console();
}

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

static int cmd_control_control_console(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&control_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, control_args.end, argv[0]);
        return 1;
    }

    const char *unit = control_args.unit->sval[0];
    const char *state = control_args.state->sval[0];

    bool bool_state = strcmp("on", state) == 0;

    if (strcmp("zvs.power", unit) == 0)
    {
        if (!AME_ANNEAL_IsAnnealing())
        {
            AME_ANNEAL_MainPowerState(bool_state);
        }
    }
    else if (strcmp("zvs.control", unit) == 0)
    {
        if (!AME_ANNEAL_IsAnnealing())
        {
            AME_ANNEAL_ControlState(bool_state);
        }
    }

    return 0;
}

/**
 * @brief Register the console commands for the pub sub module
 * 
 */
static void cmd_control_register_console(void)
{
    control_args.unit = arg_str1(NULL, NULL, "<unit>", "unit to comntrol");
    control_args.state = arg_str1(NULL, NULL, "<state>", "state to put unit in");
    control_args.end = arg_end(2);

    const esp_console_cmd_t control_cmd = {.command = "control",
                                           .help = "Control the hardware directly.\n"
                                                   "Examples:\n"
                                                   " control zvs.power on \n"
                                                   "Supported units:\n"
                                                   " zvs.power\n"
                                                   " zvs.control\n",
                                           .hint = NULL,
                                           .func = &cmd_control_control_console,
                                           .argtable = &control_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&control_cmd));
}
/*************** END OF FUNCTIONS ************************************************************************************/
