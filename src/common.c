/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * Modifications:
 * - Ported to Raspberry Pi Pico SDK
 * - Removed COINES dependency
 * - Implemented I2C access using hardware_i2c
 * - Modified delay and interface handling
 * - Removed SPI
 * - Rewrote the bme280_interface_selection -> bme280_i2c_select
 *
 * Modified by: Jakub Farník
 * Date: 22.01.2026
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "config.h"
#include "hardware/i2c.h"
#include "bme280.h"
#include "pico/stdlib.h"
#include "common.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME280_SHUTTLE_ID  UINT8_C(0x33)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to Pico SDK
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;
    i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true);
    i2c_read_blocking(I2C_PORT, dev_addr, reg_data, length, false);
    return BME280_INTF_RET_SUCCESS;
}

/*!
 * I2C write function map to Pico SDK
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;
    uint8_t buffer_to_write[length+1];
    buffer_to_write[0] = reg_addr;
    for(uint32_t i = 0; i < length; i++){
        buffer_to_write[i+1] = reg_data[i];
    }
    i2c_write_blocking(I2C_PORT, dev_addr, buffer_to_write, length+1, true);
    return BME280_INTF_RET_SUCCESS;
}


/*!
 * Delay function map to Pico SDK
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_ms(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BME280_E_COMM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BME280_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BME280_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_i2c_select(struct bme280_dev *dev)
{
    sleep_ms(100);
    int8_t rslt = BME280_OK;
    printf("inside select\n");

    if (dev != NULL)
    {
    printf("dev exists\n");
        dev_addr = BME280_I2C_ADDR_PRIM;
    printf("addr exists\n");
        dev->read = bme280_i2c_read;
    printf("read xists\n");
        dev->write = bme280_i2c_write;

    bme280_read_fptr_t read;
    printf("write dev exists\n");
        dev->intf = BME280_I2C_INTF;
    printf("intf dev exists\n");

        dev->intf_ptr = &dev_addr;

    sleep_ms(100);
    printf("intf ddelay exists\n");
        /* Configure delay in microseconds */
        dev->delay_us = bme280_delay_us;
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

