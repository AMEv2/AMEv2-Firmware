/*********************************************************************************************************************
 * Author    : Christiaan Alberts
 * Created on: 25 May 2024
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * @file   	cmd_pubsub.c
 * @brief  	pub sub console handling
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

#include "pubsub.h"

static const char *TAG = "pubsub";

/**********************************************************************************************************************
 * Module Preprocessor Constants
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Preprocessor Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Typedefs
 **********************************************************************************************************************/
/** Arguments used by 'pubsub' function */
static struct
{
    struct arg_str *topic;
    struct arg_str *type;
    struct arg_str *value;
    struct arg_end *end;
} publish_args;

/** Arguments used by 'pubsub' function */
static struct
{
    struct arg_lit *unsubscribe;
    struct arg_lit *free;
    struct arg_str *topics;
    struct arg_end *end;
} subscribe_args;

/**********************************************************************************************************************
 * Module Variable Definitions
 **********************************************************************************************************************/
static bool active = false;
static bool freeSubscriber = false;
static ps_subscriber_t *s;
static TaskHandle_t xTaskPubSub = NULL;

/**********************************************************************************************************************
 * Global Variable Definitions
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Module Function Prototypes
 **********************************************************************************************************************/
static void cmd_pubsub_Task(void *pvParameters);
static int cmd_pubsub_publish_console(int argc, char **argv);
static int cmd_pubsub_subscribe_console(int argc, char **argv);
static void cmd_pubsub_register_console(void);


/**********************************************************************************************************************
 * Global Function Definitions
 **********************************************************************************************************************/
void CMD_PUBSUB_Init(void)
{
    ESP_LOGI(TAG, "CMD Pubsub Init");
    cmd_pubsub_register_console();
}

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/
void cmd_pubsub_Task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Task Start");

    for (;;)
    {
        if ((active) && (s != NULL))
        {
            ps_msg_t *msg = ps_get(s, 5000);
            if (msg != NULL)
            {
                if (PS_IS_DBL(msg))
                {
                    printf("(%" PRIu32 ") %s: %f\n", (uint32_t)xTaskGetTickCount(), msg->topic, msg->dbl_val);
                }
                else if (PS_IS_INT(msg))
                {
                    printf("(%" PRIu32 ") %s: %lld\n", (uint32_t)xTaskGetTickCount(), msg->topic, msg->int_val);
                }
                else if (PS_IS_STR(msg))
                {
                    printf("(%" PRIu32 ") %s: %s\n", (uint32_t)xTaskGetTickCount(), msg->topic, msg->str_val);
                }
                else if (PS_IS_BOOL(msg))
                {
                    printf("(%" PRIu32 ") %s: %d\n", (uint32_t)xTaskGetTickCount(), msg->topic, msg->bool_val);
                }
            }
            ps_unref_msg(msg);

            if (freeSubscriber)
            {
                freeSubscriber = false;
                active = false;
                if (s != NULL)
                {
                    printf("freeing subscriber\n");
                    ps_free_subscriber(s);
                    s = NULL;
                }
            }
            // vTaskDelay(pdMS_TO_TICKS(interval));
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

static int cmd_pubsub_publish_console(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&publish_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, publish_args.end, argv[0]);
        return 1;
    }

    const char *topic = publish_args.topic->sval[0];
    const char *type = publish_args.type->sval[0];
    const char *values = publish_args.value->sval[0];

    if (strcmp("dbl", type) == 0)
    {
        double dblValue = strtod(values, NULL);
        PS_PUB_DBL(topic, dblValue);
    }

    return 0;
}

static int cmd_pubsub_subscribe_console(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&subscribe_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, subscribe_args.end, argv[0]);
        return 1;
    }

    if (subscribe_args.free->count != 0)
    {
        // Free reseources
        freeSubscriber = true;

        return 0;
    }

    if (subscribe_args.unsubscribe->count == 0)
    {
        // Subscribe to channel
        if (s == NULL)
        {
            s = ps_new_subscriber(10, NULL);
        }
        if (xTaskPubSub == NULL)
        {
            xTaskCreate(cmd_pubsub_Task, /* Task function. */
                        "pubsub",        /* name of task. */
                        512 * 5,         /* Stack size of task */
                        NULL,            /* parameter of the task */
                        1,               /* priority of the task */
                        &xTaskPubSub);   /* Task handle to keep track of created task */
        }

        for (uint8_t i = 0; i < subscribe_args.topics->count; i++)
        {
            if (ps_subscribe(s, subscribe_args.topics->sval[i]) == 0)
            {
                active = true;
                printf("%s subcribed\n", subscribe_args.topics->sval[i]);
            }
            else
            {
                printf("ERROR: %s subcribe fail\n", subscribe_args.topics->sval[i]);
            }
        }
    }
    else
    {
        // Unsubscribe
        if (s == NULL)
        {
            // nothing to unsubscribe from
            printf("No subscribtions active\n");
        }
        else
        {
            for (uint8_t i = 0; i < subscribe_args.topics->count; i++)
            {
                if (ps_unsubscribe(s, subscribe_args.topics->sval[i]) == 0)
                {
                    printf("%s unsubcribed\n", subscribe_args.topics->sval[i]);
                }
                else
                {
                    printf("ERROR: %s unsubcribe fail\n", subscribe_args.topics->sval[i]);
                }
            }
        }
    }
    
    return 0;
}

/**
 * @brief Register the console commands for the pub sub module
 * 
 */
static void cmd_pubsub_register_console(void)
{
    publish_args.topic = arg_str1(NULL, NULL, "<topic>", "topic to publish to");
    publish_args.type = arg_str1(NULL, NULL, "<type>", "value type. can be dbl, int, bool, str");
    publish_args.value = arg_str1("v", "value", "<value>", "value to publish");
    publish_args.end = arg_end(2);

    subscribe_args.unsubscribe = arg_lit0("u", "unsubscribe", "Unsubscribe");
    subscribe_args.free = arg_lit0("f", "free", "Free resources");
    subscribe_args.topics = arg_strn(NULL, NULL, "<topics>", 0, 5, "topics to (un)subscribe. max 5");
    subscribe_args.end = arg_end(5);

    const esp_console_cmd_t pub_cmd = {.command = "ps_pub",
                                       .help = "Publish to topic.\n"
                                               "Examples:\n"
                                               " ps_pub ame.sensor.current dbl -v 18.5 \n"
                                               " ps_pub ame.anneal.msg str -v Dropping \n",
                                       .hint = NULL,
                                       .func = &cmd_pubsub_publish_console,
                                       .argtable = &publish_args};

    const esp_console_cmd_t sub_cmd = {.command = "ps_sub",
                                       .help = "(Un)Subscribe to/from topic.\n"
                                               "Examples:\n"
                                               " ps_sub ame.sensor.current ame.anneal.msg \n"
                                               " ps_sub -u ame.sensor.current\n"
                                               " ps_sub -f\n"
                                               " ps_sub -i 1000\n"
                                               " ps_sub ame.sensor \n",
                                       .hint = NULL,
                                       .func = &cmd_pubsub_subscribe_console,
                                       .argtable = &subscribe_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&pub_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&sub_cmd));
}
/*************** END OF FUNCTIONS ************************************************************************************/
