#ifndef __CONFIG_H
#define __CONFIG_H
#include <stdint.h>

/*
 * DEBUG Functions for testing through UART
 */
//#define DEBUG
#ifdef DEBUG
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...) ((void)0)
#endif


/*
 * Hardware definitions 
 * Used with the gpio_set_dir() function
 */
#define OUTPUT true
#define INPUT false 

/* Pin definitions */
#define LED_PIN 25
#define i2c0_SDA 4
#define i2c0_SCL 5

/* i2c Protocol */
#define I2C_PORT i2c0
extern const int i2c_baudrate;

/* BME280 */
// unused remove
extern const uint8_t bme_slave_addr;
extern const uint8_t bme_id_register;


#endif
