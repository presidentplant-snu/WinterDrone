# Create core library with all core source files
add_library(core 
	ekf/ekf.c
)

# Make the core directory available as an include path
target_include_directories(core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# target_include_directories(core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/freeRTOS")

# Link with required libraries
target_link_libraries(drivers
	#mongoose
    pico_stdlib
	#pico_rand
	#pico_cyw43_arch_lwip_threadsafe_background
	#pico_cyw43_arch_lwip_sys_freertos
	#FreeRTOS-Kernel-Heap4
    #hardware_i2c
    #hardware_gpio
    #hardware_pwm
)
