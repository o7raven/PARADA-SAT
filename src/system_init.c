#include "system_init.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>


void sys_init(){
    stdio_init_all();
    printf("BOOT\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, OUTPUT);

    gpio_put(LED_PIN, 1);
    sleep_ms(5000);
    gpio_put(LED_PIN, 0);
    printf("Complete\n");
}

void i2c_initalize(){
    i2c_init(I2C_PORT, i2c_baudrate);
    gpio_set_function(i2c0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c0_SDA);
    gpio_pull_up(i2c0_SCL);
}
