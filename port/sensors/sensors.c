/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "sensors.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* I2C */
#if 1
#else
static struct repeating_timer g_timer;
void (*callback_ptr)(void);
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* I2C */
#if 1
// init
int sensor_init(void)
{
  // init
}

// write
int sensor_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop)
{
  int ret;

  ret = i2c_write_blocking(i2c_default, addr, src, len, nostop);
  if(ret == PICO_ERROR_TIMEOUT)
  {
      printf("i2c_write_blocking() PICO_ERROR_TIMEOUT\n");
      ret = -1;
  }
  else if(ret == PICO_ERROR_GENERIC)
  {
      printf("i2c_write_blocking() PICO_ERROR_GENERIC\n");
      ret = -1;
  }
  else if(ret == PICO_ERROR_NO_DATA)
  {
      printf("i2c_write_blocking() PICO_ERROR_NO_DATA\n");
      ret = -1;
  }
  else
  {
#ifdef LOG_ERR_ALL
      printf("i2c_write_blocking() ret %d\n", ret);
#endif
      ret = 0;
  }
  
  return ret;
}

// read
int sensor_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop)
{
  int ret;

  ret = i2c_read_blocking(i2c_default, addr, dst, len, nostop);
  if(ret == PICO_ERROR_TIMEOUT)
  {
      printf("i2c_read_blocking() PICO_ERROR_TIMEOUT\n");
      ret = -1;
  }
  else if(ret == PICO_ERROR_GENERIC)
  {
      printf("i2c_read_blocking() PICO_ERROR_GENERIC\n");
      ret = -1;
  }
  else if(ret == PICO_ERROR_NO_DATA)
  {
      printf("i2c_read_blocking() PICO_ERROR_NO_DATA\n");
      ret = -1;
  }
  else
  {
#ifdef LOG_ERR_ALL
      printf("i2c_read_blocking() ret %d\n", ret);
#endif
      ret = 0;
  }

  return ret;
}
#endif
