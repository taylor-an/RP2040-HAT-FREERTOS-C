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
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "sensors.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* SENSORS */

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* SENSORS */

int sensor_init(void)
{
  int ret;

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
  puts("Default I2C pins were not defined");
#else
  printf("PICO_DEFAULT_I2C %d\n", PICO_DEFAULT_I2C);
  printf("PICO_DEFAULT_I2C_SDA_PIN %d\n", PICO_DEFAULT_I2C_SDA_PIN);
  printf("PICO_DEFAULT_I2C_SCL_PIN %d\n", PICO_DEFAULT_I2C_SCL_PIN);

  // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
  ret = i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
#endif

  return 0;
}

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
  else if(ret == PICO_ERROR_NONE)
  {
      //printf("i2c_write_blocking() PICO_ERROR_NONE\n");
      ret = 0;
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
  else if(ret == PICO_ERROR_NONE)
  {
      //printf("i2c_write_blocking() PICO_ERROR_NONE\n");
      ret = 0;
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
