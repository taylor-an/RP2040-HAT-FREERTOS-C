// mpu6050.c

// WIZnet
// 2022.0506 created
// Referenced from Espressif Systems

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpu6050.h"
#include "sensors_rough.h"
#include <stdio.h>

//#define LOG_ERR_ALL

int imu_mpu6050_init(void)
{
    uint8_t mpu6050_deviceid;
    int ret;
    
    ret = mpu6050_get_deviceid(&mpu6050_deviceid);
    if(ret != 0)
    {
        printf("mpu6050_get_deviceid() failed\n");
        return -1;
    }
    printf("mpu6050_deviceid %x\n", mpu6050_deviceid);

    ret = mpu6050_wake_up();
    if(ret != 0)
    {
        printf("mpu6050_wake_up() failed\n");
        return -1;
    }

    ret = mpu6050_set_acce_fs(ACCE_FS_4G);
    if(ret != 0)
    {
        printf("mpu6050_set_acce_fs() failed\n");
        return -1;
    }
    
    ret = mpu6050_set_gyro_fs(GYRO_FS_500DPS);
    if(ret != 0)
    {
        printf("mpu6050_set_gyro_fs() failed\n");
        return -1;
    }

    return 0;
}

int mpu6050_write(uint8_t* cmd, uint32_t cmdlen)
{
    int ret;
    
    ret = sensor_write(MPU6050_I2C_ADDRESS, cmd, cmdlen, true);

    return ret;
}

int mpu6050_read(uint8_t* cmd, uint32_t cmdlen, uint8_t* data, uint32_t datalen)
{
    int ret;
    
    ret = sensor_write(MPU6050_I2C_ADDRESS, cmd, cmdlen, true);
    if(ret != 0)
    {
      return ret;
    }

    ret = sensor_read(MPU6050_I2C_ADDRESS, data, datalen, false);

    return ret;
}

int mpu6050_get_deviceid(uint8_t *deviceid)
{
    uint8_t cmd;
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd = MPU6050_WHO_AM_I;
    
    ret = mpu6050_read(&cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
    }
    
    *deviceid = data;
    return ret;
}

int mpu6050_wake_up(void)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd[0] = MPU6050_PWR_MGMT_1;
    
    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    data &= (~(1<<6));

    cmd[1] = data;
    ret = mpu6050_write(cmd, 2);
    if(ret != 0)
    {
        printf("mpu6050_write() failed\n");
        return -1;
    }
    
    return ret;
}

int mpu6050_set_acce_fs(mpu6050_acce_fs_t acce_fs)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd[0] = MPU6050_ACCEL_CONFIG;
    
    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    data &= (~(1<<3));
    data &= (~(1<<4));
    data |= (acce_fs<<3);
    
    cmd[1] = data;
    ret = mpu6050_write(cmd, 2);
    if(ret != 0)
    {
        printf("mpu6050_write() failed\n");
        return -1;
    }
    
    return ret;
}

int mpu6050_set_gyro_fs(mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd[0] = MPU6050_GYRO_CONFIG;
    
    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    data &= (~(1<<3));
    data &= (~(1<<4));
    data |= (gyro_fs<<3);
    
    cmd[1] = data;
    ret = mpu6050_write(cmd, 2);
    if(ret != 0)
    {
        printf("mpu6050_write() failed\n");
        return -1;
    }
    
    return ret;
}

int mpu6050_get_acce_fs(mpu6050_acce_fs_t *acce_fs)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd[0] = MPU6050_ACCEL_CONFIG;
    
    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    data = (data >> 3) & 0x03;
    *acce_fs = data;
    
    return ret;
}

int mpu6050_get_gyro_fs(mpu6050_gyro_fs_t *gyro_fs)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t tmp;
    
    cmd[0] = MPU6050_GYRO_CONFIG;
    
    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    data = (data >> 3) & 0x03;
    *gyro_fs = data;
    
    return ret;
}

int mpu6050_get_acce_sensitivity(float *acce_sensitivity)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t acce_fs;

    cmd[0] = MPU6050_ACCEL_CONFIG;

    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    acce_fs = (data >> 3) & 0x03;
    switch (acce_fs) {
        case ACCE_FS_2G:
            *acce_sensitivity = 16384;
            break;
        case ACCE_FS_4G:
            *acce_sensitivity = 8192;
            break;
        case ACCE_FS_8G:
            *acce_sensitivity = 4096;
            break;
        case ACCE_FS_16G:
            *acce_sensitivity = 2048;
            break;
        default:
            break;
    }
    
    return ret;
}

int mpu6050_get_gyro_sensitivity(float *gyro_sensitivity)
{
    uint8_t cmd[2];
    uint8_t data;
    int ret;
    uint8_t gyro_fs;

    cmd[0] = MPU6050_GYRO_CONFIG;

    ret = mpu6050_read(cmd, 1, &data, 1);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    gyro_fs= (data >> 3) & 0x03;
    switch (gyro_fs) {
        case GYRO_FS_250DPS:
            *gyro_sensitivity = 131;
            break;
        case GYRO_FS_500DPS:
            *gyro_sensitivity = 65.5;
            break;
        case GYRO_FS_1000DPS:
            *gyro_sensitivity = 32.8;
            break;
        case GYRO_FS_2000DPS:
            *gyro_sensitivity = 16.4;
            break;
        default:
            break;
    }
    
    return ret;
}

int mpu6050_get_raw_acce(mpu6050_raw_acce_value_t *raw_acce_value)
{
    uint8_t cmd[2];
    int ret;
    uint8_t data_rd[6] = {0};
    
    cmd[0] = MPU6050_ACCEL_XOUT_H;
    
    ret = mpu6050_read(cmd, 1, data_rd, 6);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

int mpu6050_get_raw_gyro(mpu6050_raw_gyro_value_t *raw_gyro_value)
{
    uint8_t cmd[2];
    int ret;
    uint8_t data_rd[6] = {0};
    
    cmd[0] = MPU6050_GYRO_XOUT_H;
    
    ret = mpu6050_read(cmd, 1, data_rd, 6);
    if(ret != 0)
    {
        printf("mpu6050_read() failed\n");
        return -1;
    }
    
    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

int mpu6050_get_acce(mpu6050_acce_value_t *acce_value)
{
    int ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;
    
    ret = mpu6050_get_acce_sensitivity(&acce_sensitivity);
    if(ret != 0)
    {
        printf("mpu6050_get_acce_sensitivity() failed\n");
        return ret;
    }
    
    ret = mpu6050_get_raw_acce(&raw_acce);
    if(ret != 0)
    {
        printf("mpu6050_get_raw_acce() failed\n");
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    
    return 0;
}

int mpu6050_get_gyro(mpu6050_gyro_value_t *gyro_value)
{
    int ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;
    
    ret = mpu6050_get_gyro_sensitivity(&gyro_sensitivity);
    if(ret != 0)
    {
        printf("mpu6050_get_gyro_sensitivity() failed\n");
        return ret;
    }

    ret = mpu6050_get_raw_gyro(&raw_gyro);
    if(ret != 0)
    {
        printf("mpu6050_get_raw_gyro() failed\n");
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    
    return 0;
}

int imu_mpu6050_acquire_acce(float *acce_x, float *acce_y, float *acce_z)
{
    mpu6050_acce_value_t acce = {0, 0, 0};

    if (acce_x != NULL && acce_y != NULL && acce_z != NULL) {
        if (0 == mpu6050_get_acce(&acce)) {
            *acce_x = acce.acce_x;
            *acce_y = acce.acce_y;
            *acce_z = acce.acce_z;
            return 0;
        }
    }

    printf("imu_mpu6050_acquire_acce() failed\n");
    *acce_x = 0;
    *acce_y = 0;
    *acce_z = 0;
    return -1;
}

int imu_mpu6050_acquire_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    mpu6050_gyro_value_t gyro = {0, 0, 0};

    if (gyro_x != NULL && gyro_y != NULL && gyro_z != NULL) {
        if (0 == mpu6050_get_gyro(&gyro)) {
            *gyro_x = gyro.gyro_x;
            *gyro_y = gyro.gyro_y;
            *gyro_z = gyro.gyro_z;
            return 0;
        }
    }

    printf("mpu6050_get_gyro() failed\n");
    *gyro_x = 0;
    *gyro_y = 0;
    *gyro_z = 0;
    return -1;
}


