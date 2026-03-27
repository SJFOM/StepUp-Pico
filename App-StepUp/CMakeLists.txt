cmake_minimum_required(VERSION 3.14)

######################################
# USER CONFIGURATION SECTION - START #
######################################

# EDIT THESE VARIABLES TO CUSTOMIZE YOUR BUILD

# Enable Serial comms over USB (1) or UART (0)
set(SERIAL_OVER_USB 1)

# Enable/Disable buzzer output (useful to disable when debugging to avoid noise)
set(ENABLE_BUZZER_OUTPUT 1)

####################################
# USER CONFIGURATION SECTION - END #
####################################

# Should we set app-level debugging?
if(${DO_DEBUG})
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
    pico_multicore
    hardware_adc
    hardware_pwm
    freertos_kernel
    pico_async_context_freertos
    ControlInterface
    PicoUtils
    Utils)
    
# Enable/disable STDIO via USB and UART
if(SERIAL_OVER_USB)
    pico_enable_stdio_usb(${APP_NAME} 1)
    pico_enable_stdio_uart(${APP_NAME} 0)
else()
    pico_enable_stdio_usb(${APP_NAME} 0)
    pico_enable_stdio_uart(${APP_NAME} 1)
endif()

add_compile_definitions(SERIAL_OVER_USB=${SERIAL_OVER_USB})

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})

message(STATUS "Build revision ${BUILD_NUMBER_OUTPUT}")

# Increase the amount of available shared IRQ handlers to support more peripherals (e.g. ADC, USB, TMC UART)
# Default is 4 which is not enough when we enable USB interrupts alongside the existing program IRQ handlers.
add_compile_definitions(PICO_MAX_SHARED_IRQ_HANDLERS=8)

# Forward the ENABLE_BUZZER_OUTPUT definition to the compiler so it can be used in code to conditionally
# compile out buzzer output when disabled
add_compile_definitions(ENABLE_BUZZER_OUTPUT=${ENABLE_BUZZER_OUTPUT})