// C headers
#include <stdio.h>

// Pico SDK headers
#include "bme280_defs.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// Custom headers
#include "config.h"
#include "errors.h"

// Lib headers
#include "bme280.h"
#include "common.h"

const int i2c_baudrate = 400000;
const  uint8_t bme_slave_addr = 0x76;
const uint8_t bme_id_register = 0xD0;

const uint8_t mpu_slave_addr = 0x68;
const uint8_t who_am_i_reg = 0x8;
const uint8_t mpu_id_register = 0x70;

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
static int8_t bme280_scan(uint32_t period, struct bme280_dev *dev){
    int8_t result = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < 50){
        result = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", result);
        if(status_reg & BME280_STATUS_MEAS_DONE){
            dev->delay_us(period, dev->intf_ptr);

            result = bme280_get_sensor_data(BME280_TEMP|BME280_HUM|BME280_PRESS, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", result);
#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Temperature[%d]:   %lf deg C\n", idx, comp_data.temperature);
            printf("Humidity[%d]:   %lf %%RH\n", idx, comp_data.humidity);
            printf("Pressure[%d]:  %lf Pa\n", idx, comp_data.pressure);

#else
            printf("Temperature[%d]:   %ld deg C\n", idx, (long int)comp_data.temperature);
#endif
            idx++;
        }
    }
    return result;
}

void configure_accelerometer(void){
    sleep_ms(100);
    uint8_t reg = 0x75;
    i2c_write_blocking(I2C_PORT, mpu_slave_addr, &reg, 1, true);
    uint8_t data[1];
    i2c_read_blocking(I2C_PORT, mpu_slave_addr, data, 1, false);
    
    uint8_t pwr_mgmt = 0x6B;
    uint8_t accel_config = 0x1c;
    uint8_t gyro_config = 0x1b;


    printf("chip id: 0x%X", data[0]);
    if(data[0] == 0x70){
        // wake up device
        uint8_t buffer[2] = {pwr_mgmt, 0x00};
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        buffer[1] = 0x01;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        // accel config
        buffer[0] = accel_config;
        buffer[1] = 1<<3;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        
        uint8_t acc_addr = 0x3B;
        int16_t accX;
        int16_t accY;
        int16_t accZ;
        float fx,fy,fz;

        buffer[0] = gyro_config;
        buffer[1] = 1<<3;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        buffer[0] = 0x1a;
        buffer[1] = 0x03;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        
        uint8_t gyro_addr = 0x43;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
        float gx,gy,gz;
        while(1){
            i2c_write_blocking(I2C_PORT, mpu_slave_addr, &acc_addr, 1, true);
            uint8_t accel[6];
            i2c_read_blocking(I2C_PORT, mpu_slave_addr, accel, 6, false);

            i2c_write_blocking(I2C_PORT, mpu_slave_addr, &gyro_addr, 1, true);
            uint8_t gyro[6];
            i2c_read_blocking(I2C_PORT, mpu_slave_addr, gyro, 6, false);

            accX = (accel[0]<<8)| accel[1];
            accY = (accel[2]<<8) | accel[3];
            accZ = (accel[4]<<8) | accel[5];
            fx = accX/8192.0f;
            fy = accY/8192.0f;
            fz = accZ/8192.0f;


            gyroX = (gyro[0]<<8) | gyro[1];
            gyroY = (gyro[2]<<8) | gyro[3];
            gyroZ = (gyro[4]<<8) | gyro[5];
            gx = gyroX/65.5f;
            gy = gyroY/65.5f;
            gz = gyroZ/65.5f;
            printf("Acceleration: X: %6.2f Y: %6.2f Z: %6.2f\n", fx,fy,fz);
            printf("Gyro: X: %6.2f Y: %6.2f Z: %6.2f\n", gx,gy,gz);
            sleep_ms(500);
        }


    }else{
        printf("chip not found");
    }

}


#define GPS_UART uart1
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUD 9600
void configure_gps(){
    uart_init(GPS_UART, GPS_BAUD);

    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
}
bool gps_read_line(char *buf, size_t max_len) {
    size_t i = 0;

    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);

        if (c == '\n') {
            buf[i] = '\0';
            return true;
        }

        if (i < max_len - 1) {
            buf[i++] = c;
        }
    }

    return false;
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
    
    configure_gps();
    
    char gps_line[128];

    while (true) {
        if (gps_read_line(gps_line, sizeof(gps_line))) {
            printf("GPS: %s\n", gps_line);
        }
    }
   // configure_accelerometer();

    /*
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
    result = bme280_scan(period, &dev);
    bme280_error_codes_print_result("get_temperature", result);
    */
    return 0;
    
}

