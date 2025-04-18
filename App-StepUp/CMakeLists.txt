cmake_minimum_required(VERSION 3.14)

# Should we set app-level debugging?
# NOTE Equivalent of `#define DEBUG 1`
if(${DO_DEBUG})
    add_compile_definitions(DEBUG=1)
    message(STATUS "App-side debugging enabled for ${APP_NAME}")
else()
    message(STATUS "App-side debugging disabled for ${APP_NAME}")
endif()

message(STATUS "Compiling for PCB Revision ${PCB_REVISION}")

# Include app source code file(s)
file(GLOB APP_SOURCES
    "${APP_SRC_DIRECTORY}/*.c*"
    "${APP_SRC_DIRECTORY}/src/*.c*"
    "${TMC_API_CODE_DIRECTORY}/ic/*.c*"
    "${TMC_API_CODE_DIRECTORY}/helpers/*.c*"
)

add_executable(${APP_NAME}
    ${APP_SOURCES}
)

# Link to built libraries
target_link_libraries(${APP_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS
    ControlInterface
    PicoUtils)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_NAME} 0)
pico_enable_stdio_uart(${APP_NAME} 1)

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})
