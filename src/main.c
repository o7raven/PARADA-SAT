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
#include "system_init.h"

// Chip drivers
#include "bme280_driver.h"
#include "mpu6050_driver.h"
#include "gps_driver.h"



int main(void) {
    sys_init();
    i2c_initalize();
    
    struct bme280_dev dev;
    struct bme280_data comp_data;

    bme280_initialize_full(&dev);

    mpu_data mpu_d;
    mpu6050_initialize_full();


    //gps_initialize();
    //char gps_line[128];

    while(1){
        // fix magic number
        mpu6050_load_data(&mpu_d);
        printf("\n-----------------------------\n");
        printf("[MPU] Acceleration: X: %6.2f Y: %6.2f Z: %6.2f\n",mpu_d.acc_X, mpu_d.acc_Y, mpu_d.acc_Z);
        printf("[MPU] Gyro: X: %6.2f Y: %6.2f Z: %6.2f", mpu_d.gyro_X, mpu_d.gyro_Y, mpu_d.gyro_Z);
        printf("\n-----------------------------\n");

        bme280_load_data(&dev,&comp_data);
        printf("[BME]Temperature:   %lf deg C\n", comp_data.temperature);
        printf("[BME]Humidity:   %lf %%RH\n", comp_data.humidity);
        printf("[BME]Pressure:  %lf Pa", comp_data.pressure);
        printf("\n-----------------------------\n");
       /* if (gps_read_line(gps_line, sizeof(gps_line))) {
            printf("GPS: %s\n", gps_line);
        }else {
            printf("\nGps FAILED\n");
        }
        */
        sleep_ms(450);
    }

    return 0;
    
}

