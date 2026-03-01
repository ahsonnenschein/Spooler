cmake_minimum_required(VERSION 3.13)

# Board configuration - W6100-EVB-Pico2 uses RP2350 with W6100 Ethernet
set(PICO_PLATFORM rp2350)
set(PICO_BOARD pico2)

# Fetch Pico SDK from git if not already available
set(PICO_SDK_FETCH_FROM_GIT on)
set(PICO_SDK_FETCH_FROM_GIT_TAG 2.1.0)

include(pico_sdk_import.cmake)

project(spooler C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# WIZnet chip configuration
add_compile_definitions(_WIZCHIP_=6100)
add_compile_definitions(WIZCHIP_USE_MAX_BUFFER=1)

# ioLibrary paths
set(IOLIBRARY_DIR ${CMAKE_SOURCE_DIR}/lib/ioLibrary_Driver)
set(ETHERNET_DIR ${IOLIBRARY_DIR}/Ethernet)
set(INTERNET_DIR ${IOLIBRARY_DIR}/Internet)

# ioLibrary Ethernet sources
set(ETHERNET_SOURCES
    ${ETHERNET_DIR}/wizchip_conf.c
    ${ETHERNET_DIR}/socket.c
    ${ETHERNET_DIR}/W6100/w6100.c
)

# ioLibrary Internet sources (DHCP)
set(INTERNET_SOURCES
    ${INTERNET_DIR}/DHCP/dhcp.c
)

# Port layer sources
set(PORT_SOURCES
    ${CMAKE_SOURCE_DIR}/port/w6100_spi.c
)

# Application sources
set(APP_SOURCES
    ${CMAKE_SOURCE_DIR}/src/main.c
    ${CMAKE_SOURCE_DIR}/src/modbus_server.c
    ${CMAKE_SOURCE_DIR}/src/tension_ctrl.c
    ${CMAKE_SOURCE_DIR}/src/stepper.c
    ${CMAKE_SOURCE_DIR}/src/pid.c
)

# Main executable
add_executable(spooler
    ${APP_SOURCES}
    ${PORT_SOURCES}
    ${ETHERNET_SOURCES}
    ${INTERNET_SOURCES}
)

# Generate PIO header from PIO assembly file
pico_generate_pio_header(spooler ${CMAKE_CURRENT_LIST_DIR}/src/stepper.pio)

# Include directories
target_include_directories(spooler PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/port
    ${ETHERNET_DIR}
    ${ETHERNET_DIR}/W6100
    ${INTERNET_DIR}/DHCP
)

# Link libraries
target_link_libraries(spooler
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_timer
    hardware_pio
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(spooler 1)
pico_enable_stdio_uart(spooler 0)

# Create UF2 file for flashing
pico_add_extra_outputs(spooler)
