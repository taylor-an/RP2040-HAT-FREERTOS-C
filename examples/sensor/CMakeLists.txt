set(TARGET_NAME sensor)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        ./sensors/sht3x/sht3x.c
        ./sensors/mpu6050/mpu6050.c
        ./sensors/lis2dh12/lis2dh12.c
        ./tcp/tcp.c
        )

include_directories(
        ./sensors/sht3x
        ./sensors/mpu6050
        ./sensors/lis2dh12
        ./tcp
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        FREERTOS_FILES
        ETHERNET_FILES
        IOLIBRARY_FILES
        DHCP_FILES
        DNS_FILES
        TIMER_FILES        
        hardware_i2c
        )

#pico_enable_stdio_usb(${TARGET_NAME} 1)
#pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
