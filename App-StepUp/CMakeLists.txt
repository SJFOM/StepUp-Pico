cmake_minimum_required(VERSION 3.14)

# Should we set app-level debugging?
# NOTE Equivalent of `#define DEBUG 1`
if(${DO_DEBUG})
    add_compile_definitions(DEBUG=1)
    message(STATUS "App-side debugging enabled for ${APP_NAME}")
else()
    message(STATUS "App-side debugging disabled for ${APP_NAME}")
endif()

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_NAME}")
add_compile_definitions(APP_VERSION="${APP_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Include app source code file(s)
add_executable(${APP_NAME}
    ${APP_SRC_DIRECTORY}/main.cpp
    ${APP_SRC_DIRECTORY}/src/tmc_control.cpp
    ${APP_SRC_DIRECTORY}/src/joystick_control.cpp
    ${APP_SRC_DIRECTORY}/src/buzzer_control.cpp
    ${APP_SRC_DIRECTORY}/src//utils.cpp
    ${TMC_API_CODE_DIRECTORY}/ic/TMC2300.c
    ${TMC_API_CODE_DIRECTORY}/helpers/CRC.c
)

# Link to built libraries
target_link_libraries(${APP_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_NAME} 0)
pico_enable_stdio_uart(${APP_NAME} 1)

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})
