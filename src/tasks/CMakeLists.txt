# Create drivers library with all driver source files
add_library(tasks
	wifi_server_task.c
	sensors_task.c
	pid_task.c
	shared_data.c
)

# Make the drivers directory available as an include path
target_include_directories(tasks PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(tasks PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/assets")
target_include_directories(tasks PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/freeRTOS")

# Link with required libraries
target_link_libraries(tasks
	drivers
	mongoose
    pico_stdlib
    pico_rand
	core
	#pico_cyw43_arch_lwip_threadsafe_background
	pico_cyw43_arch_lwip_sys_freertos
	FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_i2c
    hardware_gpio
    hardware_pwm
)
