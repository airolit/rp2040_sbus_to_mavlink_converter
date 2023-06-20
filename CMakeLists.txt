cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_sbus_to_mavlink_converter C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(./)

add_executable(rp2040_sbus_to_mavlink_converter
        main.c
        usb_descriptors.c
        )

target_include_directories(rp2040_sbus_to_mavlink_converter PRIVATE ${CMAKE_CURRENT_LIST_DIR}/mavlink)

# pull in common dependencies
target_link_libraries(rp2040_sbus_to_mavlink_converter pico_stdlib tinyusb_device tinyusb_board)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(rp2040_sbus_to_mavlink_converter)
