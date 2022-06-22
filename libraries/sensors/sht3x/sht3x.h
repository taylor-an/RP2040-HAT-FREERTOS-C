// sht3x.h

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

#ifndef _SHT3x_H_
#define _SHT3x_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {

    SOFT_RESET_CMD = 0x30A2,    /*!< Command to soft reset*/
    READOUT_FOR_PERIODIC_MODE = 0xE000,     /*!< Command to read Periodic*/
    READ_SERIAL_NUMBER = 0x3780,        /*!< Command to read senser number*/
    SHT3x_STOP_PERIODIC = 0x3093,       /*!< Command to break or stop periodic mode*/
    SHT3x_ART_CMD = 0x2B32,             /*!< Command to accelerated response time*/

    /*  Single Shot Data Acquisition Mode*/
    SHT3x_SINGLE_HIGH_ENABLED    = 0x2C06,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in high repeatability and Clock Stretching enabled*/
    SHT3x_SINGLE_MEDIUM_ENABLED  = 0x2C0D,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in medium repeatability and Clock Stretching enabled*/
    SHT3x_SINGLE_LOW_ENABLED     = 0x2C10,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in low repeatability and Clock Stretching enabled*/
    SHT3x_SINGLE_HIGH_DISABLED   = 0x2400,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in high repeatability and Clock Stretching disabled*/
    SHT3x_SINGLE_MEDIUM_DISABLED = 0x240B,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in medium repeatability and Clock Stretching disabled*/
    SHT3x_SINGLE_LOW_DISABLED    = 0x2416,      /*!< Command to set measure mode as Single Shot Data Acquisition mode in low repeatability and Clock Stretching disabled*/

    /*  Periodic Data Acquisition mode*/
    SHT3x_PER_0_5_HIGH   = 0x2032,      /*!< Command to set measure mode as Periodic Data Acquisition mode in high repeatability and 0.5 mps*/
    SHT3x_PER_0_5_MEDIUM = 0x2024,      /*!< Command to set measure mode as Periodic Data Acquisition mode in medium repeatability and 0.5 mps*/
    SHT3x_PER_0_5_LOW    = 0x202F,      /*!< Command to set measure mode as Periodic Data Acquisition mode in low repeatability and 0.5 mps*/
    SHT3x_PER_1_HIGH     = 0x2130,      /*!< Command to set measure mode as Periodic Data Acquisition mode in high repeatability and 1 mps*/
    SHT3x_PER_1_MEDIUM   = 0x2126,      /*!< Command to set measure mode as Periodic Data Acquisition mode in medium repeatability and 1 mps*/
    SHT3x_PER_1_LOW      = 0x212D,      /*!< Command to set measure mode as Periodic Data Acquisition mode in low repeatability and 1 mps*/
    SHT3x_PER_2_HIGH     = 0x2236,      /*!< Command to set measure mode as Periodic Data Acquisition mode in high repeatability and 2 mps*/
    SHT3x_PER_2_MEDIUM   = 0x2220,      /*!< Command to set measure mode as Periodic Data Acquisition mode in medium repeatability and 2 mps*/
    SHT3x_PER_2_LOW      = 0x222B,      /*!< Command to set measure mode as Periodic Data Acquisition mode in low repeatability and 2 mps*/
    SHT3x_PER_4_HIGH     = 0x2334,      /*!< Command to set measure mode as Periodic Data Acquisition mode in high repeatability and 4 mps*/
    SHT3x_PER_4_MEDIUM   = 0x2322,      /*!< Command to set measure mode as Periodic Data Acquisition mode in medium repeatability and 4 mps*/
    SHT3x_PER_4_LOW      = 0x2329,      /*!< Command to set measure mode as Periodic Data Acquisition mode in low repeatability and 4 mps*/
    SHT3x_PER_10_HIGH    = 0x2737,      /*!< Command to set measure mode as Periodic Data Acquisition mode in high repeatability and 10 mps*/
    SHT3x_PER_10_MEDIUM  = 0x2721,      /*!< Command to set measure mode as Periodic Data Acquisition mode in medium repeatability and 10 mps*/
    SHT3x_PER_10_LOW     = 0x272A,      /*!< Command to set measure mode as Periodic Data Acquisition mode in low repeatability and 10 mps*/

    /*  cmd for sht3x heater condition*/
    SHT3x_HEATER_ENABLE = 0x306D,   /*!< Command to enable the heater*/
    SHT3x_HEATER_DISABLED = 0x3066,   /*!< Command to disable the heater*/
} sht3x_cmd_measure_t;

typedef enum {
    SHT3x_ADDR_PIN_SELECT_VSS = 0x44, /*!< set address PIN select VSS  */
    SHT3x_ADDR_PIN_SELECT_VDD = 0x45, /*!< set address PIN select VDD  */
} sht3x_set_address_t;


int sht3x_get_humiture(float *Tem_val, float *Hum_val);
int sht3x_soft_reset(void);
int sht3x_set_measure_mode(sht3x_cmd_measure_t sht3x_measure_mode);


#ifdef __cplusplus
}
#endif

#endif


