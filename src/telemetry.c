#include "telemetry.h"
#include <string.h>
telemetry_packet_t create_packet(mpu_data *mpu_d, struct bme280_data *bme_d, 
                                    gps_data_t *gps_d){
    static uint16_t _id = 1;
    telemetry_packet_t packet = {
        _id++,
        mpu_d->acc_X*PACKET_SCALE,
        mpu_d->acc_Y*PACKET_SCALE,
        mpu_d->acc_Z*PACKET_SCALE,

        mpu_d->gyro_X*PACKET_SCALE,
        mpu_d->gyro_Y*PACKET_SCALE,
        mpu_d->gyro_Z*PACKET_SCALE,

        bme_d->temperature*PACKET_SCALE,
        bme_d->humidity*PACKET_SCALE,
        bme_d->pressure,

        gps_d->lat,
        gps_d->lon,
        
        0
    };
    packet.crc16 = crc16((uint8_t*)&packet, sizeof(packet)-sizeof(packet.crc16));
    return packet;
}

uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

#define SYNC 0xAA55
void send_packet(telemetry_packet_t *packet){
    uint8_t buff[64];
    size_t i = 0;
    const uint8_t len = sizeof(telemetry_packet_t);

    buff[i++] = (SYNC >> 8) & 0xFF;
    buff[i++] = SYNC & 0xFF;

    buff[i++] = len;
    memcpy(&buff[i], packet, len);
    i += len;

    uart_write_blocking(uart0, buff, i);
}
