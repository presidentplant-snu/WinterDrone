add_library( motor 
	motor.c 
	motor.h)

target_include_directories(motor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
