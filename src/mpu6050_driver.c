#include "mpu6050_driver.h"

void mpu6050_initialize_full(void){
    sleep_ms(chip_access_delay_ms);

    i2c_write_blocking(I2C_PORT, mpu_slave_addr, &mpu_who_am_i_reg, 1, true);
    uint8_t data[1];
    i2c_read_blocking(I2C_PORT, mpu_slave_addr, data, 1, false);
    
    printf("[+] --- MPU6050 Chip id: 0x%X", data[0]);
    if(data[0] == mpu_chip_id){
        // wake up device
        uint8_t buffer[2] = {mpu_pwr_mgmt_reg, 0x00};
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);

        buffer[1] = PLL_X_GYRO;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);

        buffer[0] = mpu_accel_config_reg;
        buffer[1] = AFS_SEL;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        

        buffer[0] = mpu_gyro_config_reg;
        buffer[1] = FS_SEL;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        buffer[0] = mpu_config_reg;
        buffer[1] = mpu_dlpf_cfg;
        i2c_write_blocking(I2C_PORT, mpu_slave_addr, buffer, 2, false);
        
    }else{
        printf("[!] --- MPU6050 Chip not found");
    }

}

void mpu6050_load_data(mpu_data* mpu_d){
    debugLED(wLED);
    int16_t i16_accX;
    int16_t i16_accY;
    int16_t i16_accZ;

    int16_t i16_gyroX;
    int16_t i16_gyroY;
    int16_t i16_gyroZ;

    i2c_write_blocking(I2C_PORT, mpu_slave_addr, &acc_reg_addr, 1, true);
    uint8_t acc_data[acc_reg_len];
    i2c_read_blocking(I2C_PORT, mpu_slave_addr, acc_data, acc_reg_len, false);

    i2c_write_blocking(I2C_PORT, mpu_slave_addr, &gyro_reg_addr, 1, true);
    uint8_t gyro_data[gyro_reg_len];
    i2c_read_blocking(I2C_PORT, mpu_slave_addr, gyro_data, gyro_reg_len, false);

    i16_accX = (acc_data[0]<<8)| acc_data[1];
    i16_accY  = (acc_data[2]<<8) | acc_data[3];
    i16_accZ = (acc_data[4]<<8) | acc_data[5];
    mpu_d->acc_X = i16_accX/ACC_LSB_SENSITIVITY;
    mpu_d->acc_Y = i16_accY/ACC_LSB_SENSITIVITY;
    mpu_d->acc_Z= i16_accZ/ACC_LSB_SENSITIVITY;

    i16_gyroX = (gyro_data[0]<<8) | gyro_data[1];
    i16_gyroY = (gyro_data[2]<<8) | gyro_data[3];
    i16_gyroZ = (gyro_data[4]<<8) | gyro_data[5];

    mpu_d->gyro_X = i16_gyroX/GYRO_LSB_SENSITIVITY;
    mpu_d->gyro_Y = i16_gyroY/GYRO_LSB_SENSITIVITY;
    mpu_d->gyro_Z = i16_gyroZ/GYRO_LSB_SENSITIVITY;
    debugLED(sLED);
}

