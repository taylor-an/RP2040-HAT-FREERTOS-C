// sht3x.c

// WIZnet
// 2022.0503 created
// Referenced from Espressif Systems

// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
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

#include "sht3x.h"
#include "sensors_rough.h"
#include <stdio.h>

//#define LOG_ERR_ALL

static int sht3x_write_cmd(sht3x_cmd_measure_t sht3x_cmd, bool nostop)
{
    uint8_t cmd_buffer[2];
    int ret;

    cmd_buffer[0] = sht3x_cmd >> 8;
    cmd_buffer[1] = sht3x_cmd;

    ret = sensor_write(SHT3x_ADDR_PIN_SELECT_VSS, cmd_buffer, 2, nostop);
    
    return ret;
}

static int sht3x_get_data(uint8_t data_len, uint8_t *data_arr)
{
    int ret;

    ret = sensor_read(SHT3x_ADDR_PIN_SELECT_VSS, data_arr, data_len, false);
    
    return ret;
}

static uint8_t CheckCrc8(uint8_t *const message, uint8_t initial_value)
{
    uint8_t  crc;
    int  i = 0, j = 0;
    crc = initial_value;

    for (j = 0; j < 2; j++) {
        crc ^= message[j];
        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;         /*!< 0x31 is Polynomial for 8-bit CRC checksum*/
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

int sht3x_get_humiture(float *Tem_val, float *Hum_val)
{
    uint8_t buff[6];
    uint16_t tem, hum;
    float Temperature = 0;
    float Humidity = 0;
    int ret;
    
    ret = sht3x_get_data(6, buff);
    
#ifdef LOG_ERR_ALL
    int i;
    for(i=0; i<6; i++)
    {
        printf("buff[%d] = %x\n", i,buff[i]);
    }
#endif
    
    if(ret != 0)
    {
        printf("sht3x_get_data() failed\n");
        return -1;
    }
    
    /* check crc */
    if (CheckCrc8(buff, 0xFF) != buff[2] || CheckCrc8(&buff[3], 0xFF) != buff[5]) {
        printf("CheckCrc8() failed\n");
        return -1;
    }

    tem = (((uint16_t)buff[0] << 8) | buff[1]);
    Temperature = (175.0 * (float)tem / 65535.0 - 45.0) ;  /*!< T = -45 + 175 * tem / (2^16-1), this temperature conversion formula is for Celsius °C */
    //Temperature= (315.0*(float)tem/65535.0-49.0) ;     /*!< T = -45 + 175 * tem / (2^16-1), this temperature conversion formula is for Fahrenheit °F */
    hum = (((uint16_t)buff[3] << 8) | buff[4]);
    Humidity = (100.0 * (float)hum / 65535.0);            /*!< RH = hum*100 / (2^16-1) */

    if ((Temperature >= -20) && (Temperature <= 125) && (Humidity >= 0) && (Humidity <= 100)) {
        *Tem_val = Temperature;
        *Hum_val = Humidity;
        return 0;                                        /*!< here is mesurement range */
    } else {
        printf("mesurement failed\n");
        return -1;
    }
}

int sht3x_soft_reset(void)
{
    int ret = sht3x_write_cmd(SOFT_RESET_CMD, false);
    return ret;
}

int  sht3x_set_measure_mode(sht3x_cmd_measure_t sht3x_measure_mode)
{
    int  ret = sht3x_write_cmd(sht3x_measure_mode, false);
    //sht3x_measure_period(true, (uint16_t *)&sht3x_measure_mode);
    return ret;
}

