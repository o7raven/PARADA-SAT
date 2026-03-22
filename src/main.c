// C headers
#include <hardware/gpio.h>
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

#include "telemetry.h"

void debugLED(uint8_t code);
int64_t led_off_callback(alarm_id_t id, void *user_data);


int main(void) {
    sys_init();
    i2c_initalize();
    
    struct bme280_dev dev;
    struct bme280_data comp_data;

    bme280_initialize_full(&dev);

    mpu_data mpu_d;
    mpu6050_initialize_full(); 


    gps_initialize();
    gps_data_t gps_d;

    while(1){
        mpu6050_load_data(&mpu_d);
        bme280_load_data(&dev,&comp_data);
        gps_update(&gps_d);
        telemetry_packet_t packet = create_packet(&mpu_d, &comp_data, &gps_d);
        send_packet(&packet);
        sleep_ms(SEND_DELAY);
    }

    return 0;
    
}

