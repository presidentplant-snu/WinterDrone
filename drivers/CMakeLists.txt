# Create drivers library with all driver source files
add_library(drivers 
    sensors/mpu6050.c 
    sensors/bmp280.c 
    motor/motor.c
    bus/i2c.c
    wifi/wifi.c
)

# Make the drivers directory available as an include path
target_include_directories(drivers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(drivers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/freeRTOS")

# Link with required libraries
target_link_libraries(drivers
	mongoose
    pico_stdlib
    pico_rand
	#pico_cyw43_arch_lwip_threadsafe_background
	pico_cyw43_arch_lwip_sys_freertos
	FreeRTOS-Kernel-Heap4
    hardware_i2c
    hardware_gpio
    hardware_pwm
)
