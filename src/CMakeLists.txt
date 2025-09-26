cmake_minimum_required(VERSION 3.31)
# Set this before your executable is defined
add_library(magenclib
   magnetic_encoder.c
)

target_include_directories(magenclib
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(magenclib
    pico_stdlib
    hardware_i2c
)

add_library(motordriver
    motor_driver.cpp
)

target_include_directories(motordriver 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(motordriver 
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_gpio
)

add_library(speedcontrol
   motor_speed_control.cpp
)

target_include_directories(speedcontrol
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(speedcontrol
    pico_stdlib
)

add_library(saganfoccontrol
    sagan_foc_control.cpp
)

target_include_directories(saganfoccontrol
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(saganfoccontrol
    pico_stdlib 
    hardware_pwm
    magenclib
)

# Add executable. Default name is the project name, version 0.1

add_executable(sagan_firmware 
    quadrature_encoder.cpp
    sagan_firmware.cpp 
)

pico_set_program_name(sagan_firmware "sagan_firmware")
pico_set_program_version(sagan_firmware "0.1")

# Generate PIO header
pico_generate_pio_header(sagan_firmware 
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(sagan_firmware 0)
pico_enable_stdio_usb(sagan_firmware 1)


# Add the standard include files to the build
target_include_directories(sagan_firmware 
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Add any user requested libraries
target_link_libraries(sagan_firmware 
    magenclib
    motordriver
    speedcontrol
    saganfoccontrol
    pico_stdlib
    pico_multicore
    pico_i2c_slave
    hardware_i2c
    hardware_pio
    hardware_timer
    hardware_clocks
)

pico_add_extra_outputs(sagan_firmware)

