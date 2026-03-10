#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include "stdint.h"
#include "mpu6050_driver.h"
#include "bme280_driver.h"
#include "gps_driver.h"

typedef struct __attribute__((packed)){
    uint16_t packet_id;

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t temp;
    int16_t humidity;
    int32_t pressure;

    int32_t gps_lat;
    int32_t gps_lon;

    uint16_t crc16;
} telemetry_packet_t;

telemetry_packet_t create_packet(mpu_data *mpu_d, struct bme280_data *bme_d, 
                                    gps_data_t *gps_d);
uint16_t crc16(const uint8_t *data, size_t len);
void send_packet(telemetry_packet_t *packet);

#endif
