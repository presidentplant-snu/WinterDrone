# Create drivers library with all driver source files
add_library(mongoose 
    mongoose.c 
)

# Make the drivers directory available as an include path
target_include_directories(mongoose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(mongoose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/freeRTOS")
target_include_directories(mongoose PUBLIC "${CMAKE_SOURCE_DIR}/config")

# Link with required libraries
target_link_libraries(mongoose
    pico_stdlib
    pico_rand
	pico_cyw43_arch_lwip_sys_freertos
	FreeRTOS-Kernel-Heap4
)

