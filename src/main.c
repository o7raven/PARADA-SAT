// C headers
#include <stdio.h>

// Pico SDK headers
#include "bme280_defs.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Custom headers
#include "config.h"
#include "errors.h"

// Lib headers
#include "bme280.h"
#include "common.h"

const int i2c_baudrate = 400000;
const uint8_t bme_slave_addr = 0x76;
const uint8_t bme_id_register = 0xD0;

uint8_t bme_available(void){
    sleep_ms(250);
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, bme_slave_addr, &bme_id_register, 1, true);
    i2c_read_blocking(I2C_PORT, bme_slave_addr, chipID, 1, false);

    if(chipID[0] != 0x60){
        return NOT_AVAILABLE;
    }
    return AVAILABLE;
}

void i2c_initalize(){
    i2c_init(I2C_PORT, i2c_baudrate);
    gpio_set_function(i2c0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c0_SDA);
    gpio_pull_up(i2c0_SCL);
}
static int8_t get_temperature(uint32_t period, struct bme280_dev *dev){
    int8_t result = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < 50){
        result = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", result);
        if(status_reg & BME280_STATUS_MEAS_DONE){
            dev->delay_us(period, dev->intf_ptr);

            result = bme280_get_sensor_data(BME280_TEMP, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", result);
#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Temperature[%d]:   %lf deg C\n", idx, comp_data.temperature);
#else
            printf("Temperature[%d]:   %ld deg C\n", idx, (long int)comp_data.temperature);
#endif
            idx++;
        }
    }
    return result;
}

int main(void) {
    stdio_init_all();
    printf("BOOT\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, OUTPUT);

    gpio_put(LED_PIN, 1);
    sleep_ms(5000);
    gpio_put(LED_PIN, 0);
    printf("Complete\n");
    i2c_initalize();
    int8_t result;
    uint32_t period;
    struct bme280_dev dev;
    struct bme280_settings settings;

    printf("1\n");
    result = bme280_i2c_select(&dev);
    bme280_error_codes_print_result("bme280_i2c_selection", result);

    printf("2\n");
    result = bme280_init(&dev);
    printf("result: %d\n", result);
    bme280_error_codes_print_result("bme280_init", result);

    printf("3\n");
    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    printf("4\n");
    result = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", result);

    printf("5\n");
    result = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    bme280_error_codes_print_result("bme280_set_sensor_mode", result);

    printf("6\n");
    result = bme280_cal_meas_delay(&period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", result);

    printf("Getting temp\n");
    result = get_temperature(period, &dev);
    bme280_error_codes_print_result("get_temperature", result);
    return 0;
    
}

