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
 * eg. Used with the gpio_set_dir() function
 */
#define OUTPUT true
#define INPUT false 
#define chip_access_delay_ms 100

/* Pin definitions */
#define LED_PIN 25
#define i2c0_SDA 4
#define i2c0_SCL 5

/* i2c Protocol */
#define I2C_PORT i2c0
static const int i2c_baudrate = 400000;


/* BME280 */

#define BME280_SAMPLE_RATE 50
// check
static const  uint8_t bme_slave_addr = 0x76;
static const uint8_t bme_id_reg = 0xD0;

/* MPU6050 */
static const uint8_t mpu_slave_addr = 0x68;
static const uint8_t mpu_chip_id = 0x70;

static const uint8_t mpu_who_am_i_reg = 0x75;
static const uint8_t mpu_pwr_mgmt_reg = 0x6B;

#define G_2 0<<3
#define G_4 1<<3
#define G_8 2<<3
#define G_16 3<<3
static const uint8_t AFS_SEL = G_4;
static const float ACC_LSB_SENSITIVITY = 8192.0f;

#define FSR_250 0 << 3
#define FSR_500 1 << 3
#define FSR_1000 2 << 3
#define FSR_2000 3 << 3
static const uint8_t FS_SEL = FSR_500;
static const float GYRO_LSB_SENSITIVITY = 65.5f;
#define PLL_X_GYRO 0x01

static const uint8_t mpu_config_reg = 0x1a;
static const uint8_t mpu_dlpf_cfg = 0x03;


static const uint8_t mpu_accel_config_reg = 0x1c;
static const uint8_t mpu_gyro_config_reg = 0x1b;

static const uint8_t acc_reg_addr = 0x3B;
static const uint8_t acc_reg_len = 6;

static const uint8_t gyro_reg_addr = 0x43;
static const uint8_t gyro_reg_len = 6;

/* GPS */
#define GPS_UART uart1
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUD 9600

#endif
