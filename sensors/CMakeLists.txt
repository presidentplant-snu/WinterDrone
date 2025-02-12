add_library( sensors 
	sensors.c 
	sensors.h)

target_include_directories(sensors PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
