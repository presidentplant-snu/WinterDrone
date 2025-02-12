add_library( sensors 
	sensors.c 
	sensors.h
	mpu.c 
	mpu.h)

target_include_directories(sensors PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(sensors
	pico_stdlib
	hardware_i2c
	hardware_gpio)
