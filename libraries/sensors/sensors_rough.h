// sensors.h

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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

int sensor_init(void) __attribute__((weak));
int sensor_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop) __attribute__((weak));
int sensor_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop) __attribute__((weak));

#ifdef __cplusplus
}
#endif

#endif


