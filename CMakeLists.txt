# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
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

add_subdirectory(drivers)

# rest of your project
add_executable(${PROJECT_NAME}
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(${PROJECT_NAME} 
	pico_stdlib 
	pico_cyw43_arch_none
	drivers
)
    

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
