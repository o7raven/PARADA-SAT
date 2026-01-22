cmake_minimum_required(VERSION 3.13)


include(pico_sdk_import.cmake)
project(parada_sat C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(parada
    src/main.c
    lib/BME280_SensorAPI/bme280.c
    src/common.c
)
target_include_directories(parada PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/lib/BME280_SensorAPI
)

pico_enable_stdio_usb(parada 0)
pico_enable_stdio_uart(parada 1)
pico_add_extra_outputs(parada)
target_link_libraries(parada pico_stdlib hardware_i2c)
