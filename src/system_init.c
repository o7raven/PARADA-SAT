#include "system_init.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <hardware/gpio.h>
#include <stdio.h>


void sys_init(){
    stdio_init_all();
    printf("BOOT\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, OUTPUT);

    gpio_put(LED_PIN, 1);

    gpio_init(sLED);
    gpio_init(wLED);
    gpio_init(eLED);

    gpio_set_dir(sLED, OUTPUT);
    gpio_set_dir(wLED, OUTPUT);
    gpio_set_dir(eLED, OUTPUT);

    gpio_put(sLED, 1);
    gpio_put(wLED, 1);
    gpio_put(eLED, 1);

    sleep_ms(1000);
    
    gpio_put(LED_PIN, 0);
    gpio_put(sLED, 0);
    gpio_put(wLED, 0);
    gpio_put(eLED, 0);

    printf("Complete\n");
}

void i2c_initalize(){
    i2c_init(I2C_PORT, i2c_baudrate);
    gpio_set_function(i2c0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c0_SDA);
    gpio_pull_up(i2c0_SCL);
}
