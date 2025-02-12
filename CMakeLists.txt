cmake_minimum_required(VERSION 3.13...3.27)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#set(ENV{PICO_SDK_PATH} "path to pico sdk")

set(PICO_BOARD pico2_w)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(WinterDrone C CXX ASM)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(sensors)
add_subdirectory(motor)

# rest of your project
add_executable(main
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main 
	pico_stdlib 
	pico_cyw43_arch_none
	sensors
	motor)
    

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main)
