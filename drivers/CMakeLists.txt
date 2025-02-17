add_library(drivers 
	sensors/mpu6050.c 
	motor/motor.c
	bus/i2c.c
	)

target_include_directories(drivers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(drivers
	pico_stdlib
	hardware_i2c
	hardware_gpio
	hardware_pwm)
