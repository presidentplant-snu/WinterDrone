cmake_minimum_required(VERSION 3.13...3.27)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# add_compile_options(-Wall -Wno-format -Wno-unused-function -Wno-uninitalized)

# Added for mongoose
add_compile_options(-fno-tree-loop-distribute-patterns)

# Configure Pico board
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM "rp2350")
set(PICO_RP2350 "1" CACHE INTERNAL "")
set(FREERTOS_KERNEL_PATH "${CMAKE_SOURCE_DIR}/src/include/FreeRTOS-Kernel/")

# Initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)
include("${CMAKE_SOURCE_DIR}/src/freeRTOS/FreeRTOS_Kernel_import.cmake")
project(WinterDrone C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(
	"${CMAKE_SOURCE_DIR}/config"
)

# Import FreeRTOS and add subdirectories
add_subdirectory(src/freeRTOS)
add_subdirectory(src/drivers)
add_subdirectory(src/mongoose)
add_subdirectory(src/core)

# Define main executable
add_executable(${PROJECT_NAME}
    src/main.c
)

# Configure include directories
target_include_directories(${PROJECT_NAME} PUBLIC 
    "${CMAKE_SOURCE_DIR}/src/"
)


# Link libraries
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib 
	#pico_cyw43_arch_lwip_threadsafe_background
	#pico_cyw43_arch_lwip_sys_freertos
	FreeRTOS-Kernel-Heap4
    drivers
	mongoose
	core
)
    
# Configure stdio
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Wait 3s for serial to start for debugging
set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)

# Set Default IP for AP
pico_configure_ip4_address(${PROJECT_NAME} PRIVATE
    CYW43_DEFAULT_IP_AP_ADDRESS 192.168.4.1
)

# Create map/bin/hex/uf2 file in addition to ELF
pico_add_extra_outputs(${PROJECT_NAME})
