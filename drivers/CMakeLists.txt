add_library(drivers 
	sensors/mpu6050.c 
	sensors/bmp280.c 
	motor/motor.c
	bus/i2c.c
	wifi/wifi.c
	)

target_include_directories(drivers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(drivers
	pico_stdlib
	pico_cyw43_arch_lwip_threadsafe_background
	hardware_i2c
	hardware_gpio
	hardware_pwm)
