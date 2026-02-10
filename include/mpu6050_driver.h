#ifndef _MPU6050_Driver
#define _MPU6050_Driver

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "config.h"


typedef struct {
    float acc_X;
    float acc_Y;
    float acc_Z;

    float gyro_X;
    float gyro_Y;
    float gyro_Z;
} mpu_data;

void mpu6050_initialize_full(void);
void mpu6050_load_data(mpu_data *mpu_d);


#endif
