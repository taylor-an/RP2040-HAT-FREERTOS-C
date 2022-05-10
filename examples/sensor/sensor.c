/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * References
 *
 * raspberrypi/pico-examples
 * https://github.com/raspberrypi/pico-examples/tree/master/i2c/bus_scan
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "dhcp.h"
#include "dns.h"

#include "timer.h"

#include "hardware/i2c.h"
#include "sht3x.h"
#include "mpu6050.h"
#include "tcp.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
// Size = STACK_SIZE * 4(WORD) ??
#define DHCP_TASK_STACK_SIZE 256
#define DHCP_TASK_PRIORITY 8

#define DNS_TASK_STACK_SIZE 128+64
#define DNS_TASK_PRIORITY 10

#define SHT3X_TASK_STACK_SIZE 128+64
#define SHT3X_TASK_PRIORITY 7
#define SHT3X_TASK_DELAY 2000

#define MPU6050_TASK_STACK_SIZE 128+64
#define MPU6050_TASK_PRIORITY 7
#define MPU6050_TASK_DELAY 2000

#define TCP_TASK_STACK_SIZE 256+64
#define TCP_TASK_PRIORITY 10
#define TCP_TASK_DELAY 2000

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_DHCP 0
#define SOCKET_DNS 3
#define SOCKET_TCP 1
#define SOCKET_TCP_PORT 5000

/* Retry count */
#define DHCP_RETRY_COUNT 5
#define DNS_RETRY_COUNT 5

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_DHCP                         // DHCP enable/disable
};
static uint8_t g_ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

/* DHCP */
static uint8_t g_dhcp_get_ip_flag = 0;

/* DNS */
static uint8_t g_dns_target_domain[] = "www.wiznet.io";
static uint8_t g_dns_target_ip[4] = {
    0,
};
static uint8_t g_dns_get_ip_flag = 0;

/* Semaphore */
static xSemaphoreHandle dns_sem = NULL;
static xSemaphoreHandle sensor_sem = NULL;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

typedef struct sht3x_message
{
    float temp;
    float humi;
} sht3x_message;

typedef struct mpu6050_message
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_message;

typedef struct sensor_message
{
    sht3x_message sht3x;
    mpu6050_message mpu6050;
} sensor_message;

static QueueHandle_t sht3x_queue;
static const int sht3x_queue_len = 10;
static QueueHandle_t mpu6050_queue;
static const int mpu6050_queue_len = 10;

static uint8_t g_send_buf[1024] = {
    0,
};

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void dhcp_task(void *argument);
void dns_task(void *argument);

void sht3x_task(void *argument);
void mpu6050_task(void *argument);

void tcp_task(void *argument);

/* Clock */
static void set_clock_khz(void);

/* DHCP */
static void wizchip_dhcp_init(void);
static void wizchip_dhcp_assign(void);
static void wizchip_dhcp_conflict(void);

/* Timer  */
static void repeating_timer_callback(void);

bool reserved_addr(uint8_t addr);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    set_clock_khz();

    stdio_init_all();

    printf("Compiled @ %s %s\n", __DATE__, __TIME__);

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    wizchip_1ms_timer_initialize(repeating_timer_callback);

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("PICO_DEFAULT_I2C %d\n", PICO_DEFAULT_I2C);
    printf("PICO_DEFAULT_I2C_SDA_PIN %d\n", PICO_DEFAULT_I2C_SDA_PIN);
    printf("PICO_DEFAULT_I2C_SCL_PIN %d\n", PICO_DEFAULT_I2C_SCL_PIN);


    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
#endif

    sht3x_queue = xQueueCreate(sht3x_queue_len, sizeof(sht3x_message));
    mpu6050_queue = xQueueCreate(mpu6050_queue_len, sizeof(mpu6050_message));

    #if 1
    BaseType_t ret;
    ret = xTaskCreate(dhcp_task, "DHCP_Task", DHCP_TASK_STACK_SIZE, NULL, DHCP_TASK_PRIORITY, NULL);
    printf("xTaskCreate(dhcp_task) = %d\n", ret);
    #endif

    #if 0
    ret = xTaskCreate(dns_task, "DNS_Task", DNS_TASK_STACK_SIZE, NULL, DNS_TASK_PRIORITY, NULL);
    printf("xTaskCreate(dns_task) = %d\n", ret);
    #endif
    
    #if 1
    ret = xTaskCreate(sht3x_task, "SHT3X_Task", SHT3X_TASK_STACK_SIZE, NULL, SHT3X_TASK_PRIORITY, NULL);
    printf("xTaskCreate(sht3x_task) = %d\n", ret);
    #endif
    
    #if 1
    ret = xTaskCreate(mpu6050_task, "MPU6050_Task", MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY, NULL);
    printf("xTaskCreate(mpu6050_task) = %d\n", ret);
    #endif

    #if 1
    ret = xTaskCreate(tcp_task, "TCP_Task", TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, NULL);
    printf("xTaskCreate(tcp_task) = %d\n", ret);
    #endif

    dns_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    sensor_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);

    printf("Start sensor\n");

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

void sht3x_task(void *argument)
{
#if 1
    // Every run a software reset
#define SH31_RESET
#endif
        
#if 1
    // Single Mode or Periodic Mode
#define SH31_SINGLE
#endif

#if 0
 #define DEBUG_SHT3X
#endif
    UBaseType_t uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    bool init_sht3x = false;
    bool set_periodic = false;
    float Tem_val, Hum_val;
    int ret;
    uint32_t i = 0;
    sht3x_message sht3x_data;
    
    printf("start sht3x_task\n");
    
    while(1)
    {
        xSemaphoreTake(sensor_sem, portMAX_DELAY);
        taskENTER_CRITICAL();
        if(init_sht3x == false)
        {
#ifdef SH31_RESET
            #ifdef DEBUG_SHT3X
            printf("Run sht3x_soft_reset()\n");
            #endif
            ret = sht3x_soft_reset();
            if(ret == 0)
            {
                i2c_init(i2c_default, 100 * 1000);
                init_sht3x = true;
            }
            else
            {
                printf("sht3x_soft_reset() failed\n");
            }
#else
            init_sht3x = true;
#endif
        }
        else
        {
#ifdef SH31_SINGLE
            if(set_periodic == false)
            {
                #ifdef DEBUG_SHT3X
                printf("Run sht3x_set_measure_mode() Single\n");
                #endif
                
                // Single
                ret = sht3x_set_measure_mode(SHT3x_SINGLE_MEDIUM_DISABLED);
                if(ret == 0)
                {
                    #ifdef DEBUG_SHT3X
                    printf("sht3x_set_measure_mode() success\n");
                    #endif
                    //set_periodic = true; // It should be comment on sigle mode
                }
                else
                {
                    printf("sht3x_set_measure_mode() failed\n");
                }
            }
#else
            if(set_periodic == false)
            {
                #ifdef DEBUG_SHT3X
                printf("Run sht3x_set_measure_mode() Periodic\n");
                #endif
                
                // Periodic
                ret = sht3x_set_measure_mode(SHT3x_PER_2_MEDIUM);
                if(ret == 0)
                {
                    #ifdef DEBUG_SHT3X
                    printf("sht3x_set_measure_mode() success\n");
                    #endif
                    set_periodic = true;
                }
                else
                {
                    printf("sht3x_set_measure_mode() failed\n");
                }
            }
#endif
            #ifdef DEBUG_SHT3X
            printf("Run sht3x_get_humiture()\n");
            #endif
            ret = sht3x_get_humiture(&Tem_val, &Hum_val);
            if(ret == 0)
            {
                #ifdef DEBUG_SHT3X
                printf("sht3x_get_humiture() success %d\n", i++);
                printf("Tem %.2f°C\n", Tem_val);
                printf("Hum %.2f%%\n", Hum_val);
                #endif
                
                sht3x_data.temp = Tem_val;
                sht3x_data.humi = Hum_val;
                
                if(xQueueSend(sht3x_queue, (void *)&sht3x_data, 0) != pdTRUE)
                {
                    printf("xQueueSend(sht3x_queue) failed\n");
                }
            }
            else
            {
                printf("sht3x_get_humiture() failed\n");
            }
            #ifdef DEBUG_SHT3X
            printf("Measure After 10000ms\n");
            printf("\n");
            #endif
        }
        taskEXIT_CRITICAL();
        xSemaphoreGive(sensor_sem);
        vTaskDelay(SHT3X_TASK_DELAY);
        #if 0
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("sht3x_task stack remains %ld Word\n", uxHighWaterMark);
        #endif

    }
}

void mpu6050_task(void *argument)
{
#if 0
#define DEBUG_MPU6050
#endif
    UBaseType_t uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    bool init_mpu6050 = false;
    int ret;
    uint32_t i = 0;
    uint32_t j = 0;
    mpu6050_message mpu6050_data;
        
    printf("start mpu6050_task\n");
    
    while(1)
    {
        xSemaphoreTake(sensor_sem, portMAX_DELAY);
        taskENTER_CRITICAL();
        if(init_mpu6050 == false)
        {
            ret = imu_mpu6050_init();
            if(ret != 0)
            {
                printf("imu_mpu6050_init() failed\n");
            }
            else
            {
                init_mpu6050 = true;
            }
        }
        else
        {
            float acce_x;
            float acce_y;
            float acce_z;
            
            float gyro_x;
            float gyro_y;
            float gyro_z;
            
            ret = imu_mpu6050_acquire_acce(&acce_x, &acce_y, &acce_z);
            if(ret != 0)
            {
                printf("imu_mpu6050_acquire_acce() failed");
            }
            else
            {
                #ifdef DEBUG_MPU6050
                printf("accel.xyz %d\n", i++);
                printf("%f %f %f\n", acce_x, acce_y, acce_z);
                printf("\n");
                #endif
            }
            
            ret = imu_mpu6050_acquire_gyro(&gyro_x, &gyro_y, &gyro_z);
            if(ret != 0)
            {
                printf("imu_mpu6050_acquire_gyro() failed");
            }
            else
            {
                #ifdef DEBUG_MPU6050
                printf("gyro.xyz %d\n", j++);
                printf("%f %f %f\n", gyro_x, gyro_y, gyro_z);
                printf("\n");
                #endif
            }

            mpu6050_data.accel_x = acce_x;
            mpu6050_data.accel_y = acce_y;
            mpu6050_data.accel_z = acce_z;
            
            mpu6050_data.gyro_x = gyro_x;
            mpu6050_data.gyro_y = gyro_y;
            mpu6050_data.gyro_z = gyro_z;
            
            if(xQueueSend(mpu6050_queue, (void *)&mpu6050_data, 0)  != pdTRUE)
            {
                printf("xQueueSend(sht3x_queue) failed\n");
            }
        }
        taskEXIT_CRITICAL();
        xSemaphoreGive(sensor_sem);
        vTaskDelay(MPU6050_TASK_DELAY);
        #if 0
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("mpu6050_task stack remains %ld Word\n", uxHighWaterMark);
        #endif
    }
}

void dhcp_task(void *argument)
{
    UBaseType_t uxHighWaterMark;
    
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    int retval = 0;
    uint8_t link;
    uint16_t len = 0;
    uint32_t dhcp_retry = 0;

    if (g_net_info.dhcp == NETINFO_DHCP) // DHCP
    {
        wizchip_dhcp_init();
    }
    else // static
    {
        network_initialize(g_net_info);
        
        /* Get network information */
        print_network_information(g_net_info);
        
        while (1)
        {
            vTaskDelay(1000 * 1000);
        }
    }
    
    while (1)
    {
        link = wizphy_getphylink();

        if (link == PHY_LINK_OFF)
        {
            printf("PHY_LINK_OFF\n");

            DHCP_stop();

            while (1)
            {
                link = wizphy_getphylink();

                if (link == PHY_LINK_ON)
                {
                    wizchip_dhcp_init();

                    dhcp_retry = 0;

                    break;
                }

                vTaskDelay(1000);
            }
        }

        retval = DHCP_run();

        if (retval == DHCP_IP_LEASED)
        {
            if (g_dhcp_get_ip_flag == 0)
            {
                dhcp_retry = 0;

                printf(" DHCP success\n");

                g_dhcp_get_ip_flag = 1;

                xSemaphoreGive(dns_sem);
                xSemaphoreGive(sensor_sem);
            }
        }
        else if (retval == DHCP_FAILED)
        {
            g_dhcp_get_ip_flag = 0;
            dhcp_retry++;

            if (dhcp_retry <= DHCP_RETRY_COUNT)
            {
                printf(" DHCP timeout occurred and retry %d\n", dhcp_retry);
            }
        }

        if (dhcp_retry > DHCP_RETRY_COUNT)
        {
            printf(" DHCP failed\n");

            DHCP_stop();

            while (1)
            {
                vTaskDelay(1000 * 1000);
            }
        }

        vTaskDelay(10);
        #if 0
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("dhcp_task stack remains %ld Word\n", uxHighWaterMark);
        #endif
    }
}

void dns_task(void *argument)
{
    UBaseType_t uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    uint8_t dns_retry;

    printf("start dns_task\n");
    
    while (1)
    {
        xSemaphoreTake(dns_sem, portMAX_DELAY);
        DNS_init(SOCKET_DNS, g_ethernet_buf);

        dns_retry = 0;

        while (1)
        {
            if (DNS_run(g_net_info.dns, g_dns_target_domain, g_dns_target_ip) > 0)
            {
                printf(" DNS success\n");
                printf(" Target domain : %s\n", g_dns_target_domain);
                printf(" IP of target domain : %d.%d.%d.%d\n", g_dns_target_ip[0], g_dns_target_ip[1], g_dns_target_ip[2], g_dns_target_ip[3]);

                break;
            }
            else
            {
                dns_retry++;

                if (dns_retry <= DNS_RETRY_COUNT)
                {
                    printf(" DNS timeout occurred and retry %d\n", dns_retry);
                }
                vTaskDelay(10);
            }

            if (dns_retry > DNS_RETRY_COUNT)
            {
                printf(" DNS failed\n");

                break;
            }
        }
        printf("done\n");
        #if 0
        xSemaphoreGive(dns_sem);
        #endif
        vTaskDelay(30*1000);
        #if 0
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("dns_task stack remains %ld Word\n", uxHighWaterMark);
        #endif
    }
}

void tcp_task(void *argument)
{
    UBaseType_t uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    int ret;
    bool init_tcp = false;
    int32_t len;
    bool got_sht3x = false;
    bool got_mpu6050 = false;
    sensor_message sensor_data;
    int32_t sent_cnt;
    int32_t wait_cnt;
    
    printf("start tcp_task\n");
    
    sensor_data.sht3x.temp = 0;
    sensor_data.sht3x.humi = 0;
    sensor_data.mpu6050.accel_x = 0;
    sensor_data.mpu6050.accel_y = 0;
    sensor_data.mpu6050.accel_z = 0;
    sensor_data.mpu6050.gyro_x = 0;
    sensor_data.mpu6050.gyro_y = 0;
    sensor_data.mpu6050.gyro_z = 0;
    
    wait_cnt = 0;
    
    while (1)
    {
        if(g_dhcp_get_ip_flag == 1)
        {
            if(init_tcp == false)
            {
                ret = tcps(SOCKET_TCP, SOCKET_TCP_PORT);
                if(ret == 1)
                {
                    init_tcp = true;
                    sent_cnt = 0;
                    wait_cnt = 0;
                }
                else if(ret == 2)
                {
                    printf("TCP Server Waiting %d Clients IP %d.%d.%d.%d @ %d\n", wait_cnt++, g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3], SOCKET_TCP_PORT);
                }
                else
                {
                    init_tcp = false;
                }
            }
            else
            {
                memset(g_send_buf, 0, sizeof(g_send_buf));
            }
            
            if(got_sht3x == false)
            {
                if (xQueueReceive(sht3x_queue, (void *)&sensor_data.sht3x, 0) == pdTRUE)
                {
                    #if 0
                    printf("temp = %f\n", sensor_data.sht3x.temp);
                    printf("humi = %f\n", sensor_data.sht3x.humi);
                    #endif
                    got_sht3x = true;
                }
            }
            
            if(got_mpu6050 == false)
            {
                if (xQueueReceive(mpu6050_queue, (void *)&sensor_data.mpu6050, 0) == pdTRUE)
                {
                    #if 0
                    printf("accel.x = %f\n", sensor_data.mpu6050.accel_x);
                    printf("accel.y = %f\n", sensor_data.mpu6050.accel_y);
                    printf("accel.z = %f\n", sensor_data.mpu6050.accel_z);
                    
                    printf("gyro.x = %f\n", sensor_data.mpu6050.gyro_x);
                    printf("gyro.y = %f\n", sensor_data.mpu6050.gyro_y);
                    printf("gyro.z = %f\n", sensor_data.mpu6050.gyro_z);
                    #endif
                    got_mpu6050 = true;
                }
            }
            
            if(got_sht3x == true && got_mpu6050 == true)
            {
                sprintf(g_send_buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", sensor_data.sht3x.temp, sensor_data.sht3x.humi,
                sensor_data.mpu6050.accel_x, sensor_data.mpu6050.accel_y, sensor_data.mpu6050.accel_z,
                sensor_data.mpu6050.gyro_x, sensor_data.mpu6050.gyro_y, sensor_data.mpu6050.gyro_z
                );
                len = strlen(g_send_buf);
                
                if(init_tcp == true)
                {
                    printf("send to client %d %s", sent_cnt++, g_send_buf);
                    ret = tcps_send(SOCKET_TCP, g_send_buf, len);
                    if(ret < 0)
                    {
                        printf("send error : %d\n", ret);
                        init_tcp = false;
                    }
                    else if(ret != 0)
                    {
                        if(ret != len)
                        {
                            printf("sent %d/%d\n", ret, len);
                        }
                    }
                }
                else
                {
                    printf("got datas %s", g_send_buf);
                }
            }
            got_sht3x = false;
            got_mpu6050 = false;
        }
        vTaskDelay(TCP_TASK_DELAY);
        #if 0
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("tcp_task stack remains %ld Word\n", uxHighWaterMark);
        #endif
    }
}

/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

/* DHCP */
static void wizchip_dhcp_init(void)
{
    printf(" DHCP client running\n");

    DHCP_init(SOCKET_DHCP, g_ethernet_buf);

    reg_dhcp_cbfunc(wizchip_dhcp_assign, wizchip_dhcp_assign, wizchip_dhcp_conflict);

    g_dhcp_get_ip_flag = 0;
}

static void wizchip_dhcp_assign(void)
{
    getIPfromDHCP(g_net_info.ip);
    getGWfromDHCP(g_net_info.gw);
    getSNfromDHCP(g_net_info.sn);
    getDNSfromDHCP(g_net_info.dns);

    g_net_info.dhcp = NETINFO_DHCP;

    /* Network initialize */
    network_initialize(g_net_info); // apply from DHCP

    print_network_information(g_net_info);
    printf(" DHCP leased time : %ld seconds\n", getDHCPLeasetime());
}

static void wizchip_dhcp_conflict(void)
{
    printf(" Conflict IP from DHCP\n");

    // halt or reset or any...
    while (1)
    {
        vTaskDelay(1000 * 1000);
    }
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;

    if (g_msec_cnt >= 1000 - 1)
    {
        g_msec_cnt = 0;

        DHCP_time_handler();
        DNS_time_handler();
    }
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

