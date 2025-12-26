cmake_minimum_required(VERSION 3.24)

set(CMAKE_C_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(wirelessaudio_fw)

pico_sdk_init()

set(LDAC_PATH ${CMAKE_CURRENT_LIST_DIR}/libldac)
set(DOGM_PATH ${CMAKE_CURRENT_LIST_DIR}/DogGraphicDisplay-pico)

if(DEFINED ENV{FDK_AAC_PATH})
    set(FDK_AAC_INCLUDES $ENV{FDK_AAC_PATH}/libAACenc/include)
    add_subdirectory($ENV{FDK_AAC_PATH})
endif()

# Add libldac
add_library(ldac
    ${LDAC_PATH}/src/ldacBT.c
    ${LDAC_PATH}/src/ldaclib.c
)
target_include_directories(ldac PRIVATE 
    ${LDAC_PATH}/inc 
    ${LDAC_PATH}/src
)

# Add DogM Graphics Library
add_library(dogm
    ${DOGM_PATH}/src/DogGraphicDisplay.cpp
)
target_include_directories(dogm PRIVATE
    ${DOGM_PATH}/src
)

if(PICO_PLATFORM STREQUAL "rp2040")
    # No hardware floating point, turn on fixed-point for LDAC
    target_compile_definitions(ldac PUBLIC "-D_32BIT_FIXED_POINT")
    message(WARNING "RP2040 detected, no hardware floating point support. Enabling LDAC fixed point option.")
endif()

# Add i2s library 
add_library(rp2040_i2s
    rp2040_i2s_example/i2s.c
)
target_link_libraries(rp2040_i2s PRIVATE
    pico_stdlib
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_clocks
)
pico_generate_pio_header(rp2040_i2s ${CMAKE_CURRENT_LIST_DIR}/rp2040_i2s_example/i2s.pio)

# Main firmware executable
add_executable(wirelessaudio_fw_pico
    src/main.c
    src/adc_controller.c
    src/bt/bt_controller.c
    src/bt/codecs.c
    src/bt/sbc.c
    src/bt/a2dp_source.c
    src/dac_controller.c
    src/ui_controller.cpp
    src/util.c
    ${DOGM_PATH}/src/arduino_pico.cpp
)
target_include_directories(wirelessaudio_fw_pico PRIVATE
    include
    rp2040_i2s_example
    ${DOGM_PATH}/src
    ${LDAC_PATH}/inc
    ${FDK_AAC_INCLUDES}
)
target_link_libraries(wirelessaudio_fw_pico
    # i2s library
    rp2040_i2s
    # libldac library
    ldac
    # Dog Graphic Display Lib
    dogm
    # Pico Standard Library stuff:
    pico_stdlib
    pico_btstack_classic
    pico_btstack_sbc_encoder
    pico_btstack_cyw43
    pico_cyw43_arch_none
    pico_multicore
    hardware_i2c
    hardware_spi
)

if (DEFINED FDK_AAC_INCLUDES)
    target_link_libraries(wirelessaudio_fw_pico fdk-aac)
    target_compile_definitions(wirelessaudio_fw_pico PUBLIC "-DENABLE_AAC")
endif()

# Redirect printf to UART over USB
pico_enable_stdio_usb(wirelessaudio_fw_pico 1)
pico_enable_stdio_uart(wirelessaudio_fw_pico 0)

# Generate hex file
pico_add_extra_outputs(wirelessaudio_fw_pico)
