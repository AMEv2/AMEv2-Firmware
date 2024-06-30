/*
 * u8g2_esp32_hal.h
 *
 *  Created on: Feb 12, 2017
 *      Author: kolban
 */

#ifndef U8G2_ESP32_HAL_H_
#define U8G2_ESP32_HAL_H_
#include "u8g2.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#define U8G2_ESP32_HAL_UNDEFINED GPIO_NUM_NC

/** @public
 * HAL configuration structure.
 */
typedef struct
{
    union
    {
        /* SPI settings. */
        struct
        {
            /* GPIO num for clock. */
            gpio_num_t clk;
            /* GPIO num for SPI mosi. */
            gpio_num_t mosi;
            /* GPIO num for SPI slave/chip select. */
            gpio_num_t cs;
        } spi;
        /* I2C settings. */
        struct
        {
            /* GPIO num for I2C data. */
            gpio_num_t sda;
            /* GPIO num for I2C clock. */
            gpio_num_t scl;
        } i2c;
    } bus;
    /* GPIO num for reset. */
    gpio_num_t reset;
    /* GPIO num for DC. */
    gpio_num_t dc;
} u8g2_esp32_hal_t;

/**
 * Construct a default HAL configuration with all fields undefined.
 */
#define U8G2_ESP32_HAL_DEFAULT                                            \
    {                                                                     \
        .bus = {.spi = {.clk = U8G2_ESP32_HAL_UNDEFINED,                  \
                        .mosi = U8G2_ESP32_HAL_UNDEFINED,                 \
                        .cs = U8G2_ESP32_HAL_UNDEFINED}},                 \
        .reset = U8G2_ESP32_HAL_UNDEFINED, .dc = U8G2_ESP32_HAL_UNDEFINED \
    }

void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
#endif /* U8G2_ESP32_HAL_H_ */
