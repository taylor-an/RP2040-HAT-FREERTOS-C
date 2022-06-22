set(TARGET_NAME sensor_monitor)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        ./tcp/tcp.c
        )

include_directories(
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
        SENSORS_FILES
        SENSOR_SHT3X_FILES
        SENSOR_MPU6050_FILES
        SENSOR_LIS2DH12_FILES
        )

#pico_enable_stdio_usb(${TARGET_NAME} 1)
#pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
